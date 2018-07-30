import sys
import numpy as np
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_road_pb2
from shapely.geometry import LineString, Point
import math

class Point3D:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
		
class Lane:
	def __init__(self, id, map):
		self.lane = map.lane.add()
		self.lane.id.id = str(id)
		self._id = id

	def setLength(self, length):
		self.lane.length = length

	def setSpeedLimit(self, speed_limit):
		self.lane.speed_limit = speed_limit

	def addOverlap(self, id):
		self.lane.overlap_id.add().id = str(id)

	def addPredecessor(self, id):
		self.lane.predecessor_id.add().id = str(id)

	def addSuccessor(self, id):
		self.lane.successor_id.add().id = str(id)

	def addLeftNeighborForward(self, id):
		self.lane.left_neighbor_forward_lane_id.add().id = str(id)

	def addRightNeoghborForward(self, id):
		self.lane.right_neighbor_forward_lane_id.add().id = str(id)

	def setType(self, type):
		self.lane.type = type

	def setTurn(self, turn):
		self.lane.turn = turn

	def addLeftLaneBoundary(self, length, virtual, s, x, y, z, heading, type):
		#boundary type params
		boundary_type = self.lane.left_boundary.boundary_type.add()
		boundary_type.s = s #double
		boundary_type.types.append(type) # map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW
		self.lane.left_boundary.length = length #double
		self.lane.left_boundary.virtual = virtual #bool
		
		#curve segment params
		left_boundary = self.lane.left_boundary.curve.segment.add()
		left_boundary.s = s
		left_boundary.heading = heading
		left_boundary.length = length
		left_boundary.start_position.x = x
		left_boundary.start_position.y = y
		left_boundary.start_position.z = z

		return left_boundary
		
	def addRightLaneBoundary(self, length, virtual, s, x, y, z, heading, type):
		#boundary type params
		boundary_type = self.lane.right_boundary.boundary_type.add()
		boundary_type.s = s #double
		boundary_type.types.append(type) # map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW
		self.lane.right_boundary.length = length #double
		self.lane.right_boundary.virtual = virtual #bool
		
		#curve segment params
		right_boundary = self.lane.right_boundary.curve.segment.add()
		right_boundary.s = s
		right_boundary.heading = heading
		right_boundary.length = length
		right_boundary.start_position.x = x
		right_boundary.start_position.y = y
		right_boundary.start_position.z = z
		return right_boundary
	
	def addCentralCurve(self, length, s, x, y, z, heading):
		central_curve = self.lane.central_curve.segment.add()
		central_curve.s = s
		central_curve.heading = heading
		central_curve.length = length
		central_curve.start_position.x = x
		central_curve.start_position.y = y
		central_curve.start_position.z = z
		return central_curve
		
		
	def addLeftSampleAssoc(self, s, width):
		ls = self.lane.left_sample.add()
		ls.s = s
		ls.width = width

	def addRightSampleAssoc(self, s, width):
		rs = self.lane.right_sample.add()
		rs.s = s
		rs.width = width

	def addLeftRoadSampleAssoc(self, s, width):
		lrs = self.lane.left_road_sample.add()
		lrs.s = s
		lrs.width = width

	def addRightRoadSampleAssoc(self, s, width):
		rrs = self.lane.right_road_sample.add()
		rrs.s = s
		rrs.width = width

	def addLeftForwardNeighbor(self,id):
		self.lane.left_neighbor_forward_lane_id.add().id = str(id)

	def addLeftReverseNeighbor(self,id):
		self.lane.left_neighbor_reverse_lane_id.add().id = str(id)

	def addRightForwardNeighbor(self,id):
		self.lane.right_neighbor_forward_lane_id.add().id = str(id)

	def addRightReverseNeighbor(self,id):
		self.lane.right_neighbor_reverse_lane_id.add().id = str(id)

	def laneSampling(self, points, width, left_boundary, right_boundary, central_curve):
		path = LineString(points)
		length = int(path.length)
		leftPoly = []
		rightPoly = []
		for i in range(length - 1):
			left_bound_point = left_boundary.line_segment.point.add()
			right_bound_point = right_boundary.line_segment.point.add()
			central_point = central_curve.line_segment.point.add()

			if i > 0 :
				p = path.interpolate(i - 1)
				p2 = path.interpolate(i - 1 + 0.5)
			else:
				p = path.interpolate(i)
				p2 = path.interpolate(i + 0.5)
			distance = width / 2.0
			lp, rp = self.convert(p, p2, distance)

			left_bound_point.x = lp[0]
			left_bound_point.y = lp[1]
			right_bound_point.x = rp[0]
			right_bound_point.y = rp[1]
			central_point.x = p.x
			central_point.y = p.y
			leftPoly.append((lp[0], lp[1]))
			rightPoly.append((rp[0], rp[1]))

			left_sample = self.lane.left_sample.add()
			left_sample.s = i + 1
			left_sample.width = width / 2.0

			right_sample = self.lane.right_sample.add()
			right_sample.s = i + 1
			right_sample.width = width / 2.0
		
		self._polygon = leftPoly + list(reversed(rightPoly)) 

	def convert(self, p, p2, distance):
		delta_y = p2.y - p.y
		delta_x = p2.x - p.x
		
		left_angle = math.atan2(delta_y, delta_x) + math.pi / 2.0
		right_angle = math.atan2(delta_y, delta_x) - math.pi / 2.0
		
		lp = []
		lp.append(p.x + (math.cos(left_angle) * distance))
		lp.append(p.y + (math.sin(left_angle) * distance))

		rp = []
		rp.append(p.x + (math.cos(right_angle) * distance))
		rp.append(p.y + (math.sin(right_angle) * distance))
		return lp, rp
	
	def getPolygon(self):
		return self._polygon
	
	def getID(self):
		return self._id
	
	def add(self, id, length, speedLimit, turnType, laneType, CCs, CCx, CCy, CCz, 
			LBs, LBx, LBy, LBz, RBs, RBx, RBy, RBz, heading, boundType, points, virtual, width):
		lane.setLength(length)
		lane.setSpeedLimit(speedLimit)
		lane.setTurn(turnType)
		lane.setType(laneType)
		central_curve = lane.addCentralCurve(length, CCs, CCx, CCy, CCz, heading)
		left_boundary = lane.addLeftLaneBoundary(length, virtual, LBs, LBx, LBy, LBz, heading, boundType)
		right_boundary = lane.addRightLaneBoundary(length, virtual, RBs, RBx, RBy, RBz, heading, boundType)
		lane.laneSampling(points, width, left_boundary, right_boundary, central_curve)
		poly = self.getPolygon()
		
	
class Road:
	def __init__(self, id, map):
		self.road = map.road.add()
		self.road.id.id = str(id)
		self.id = _id

	def addSection(self, id):
		self.section = self.road.section.add()
		self.section.id.id = str(id)

	def addLanes2Section(self, lanes):
		for i in range(len(lanes)):
			self.section.lane_id.add().id = str(lanes[i])

	def addJunction(self, id):
		self.road.junction_id.id = str(id)

	def addRoadBoundary(self, type, s, x, y, z, heading, length, points):
		edge = self.section.boundary.outer_polygon.edge.add()
		edge.type = type
		segment = edge.curve.segment.add()
		segment.s = s
		segment.start_position.x = x
		segment.start_position.y = y
		segment.start_position.z = z
		segment.heading = heading
		segment.length = length
		point = []
		for i in range(len(points)):
			p = segment.line_segment.point.add()
			point.append(p)
			point[i].x = points[i].x
			point[i].y = points[i].y
			point[i].z = points[i].z
		self.polygon = np.array(point)
	
	def getID(self):
		return self._id

#testing methods
map = map_pb2.Map()
lane = Lane(1, map)
points2D = []
for i in range(10):
 	points2D.append((i,i+1))
lane.add(1, 100, 50, map_lane_pb2.Lane.NO_TURN, map_lane_pb2.Lane.CITY_DRIVING,
		 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW, points2D, True, 3.3)


