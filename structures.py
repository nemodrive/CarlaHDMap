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

	def addLeftLaneBoundary(self, length, virtual, s, x, y, z, heading, type, points):
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
		
		#curve segment point set
		point = []
		for i in range(len(points)):
			p = left_boundary.line_segment.point.add()
			point.append(p)
			point[i].x = points[i].x
			point[i].y = points[i].y
			point[i].z = points[i].z
		self.leftPolySide = np.array(point)
		return left_boundary
		
	def addRightLaneBoundary(self, length, virtual, s, x, y, z, heading, type, points):
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
		
		#curve segment point set
		point = []
		for i in range(len(points)):
			p = right_boundary.line_segment.point.add()
			point.append(p)
			point[i].x = points[i].x
			point[i].y = points[i].y
			point[i].z = points[i].z
		self.rightPolySide = np.array(point)
		return right_boundary
	
	def addCentralCurve(self, length, s, x, y, z, heading, points):
		central_curve = self.lane.central_curve.segment.add()
		central_curve.s = s
		central_curve.heading = heading
		central_curve.length = length
		central_curve.start_position.x = x
		central_curve.start_position.y = y
		central_curve.start_position.z = z
		
		#curve segment point set
		point = []
		for i in range(len(points)):
			p = central_curve.line_segment.point.add()
			point.append(p)
			point[i].x = points[i].x
			point[i].y = points[i].y
			point[i].z = points[i].z
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

			left_sample = self.lane.left_sample.add()
			left_sample.s = i + 1
			left_sample.width = width / 2.0

			right_sample = self.lane.right_sample.add()
			right_sample.s = i + 1
			right_sample.width = width / 2.0

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
		self.polygon = np.append(self.leftPolySide, np.flipud(self.rightPolySide))
		return self.polygon
	
class Road:
	def __init__(self, id, map):
		self.road = map.road.add()
		self.road.id.id = str(id)

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
		
#testing class methods..
		
map = map_pb2.Map()
laneObj = Lane(1, map)
points = []
points2D = []
for i in range(10):
	points.append(Point3D(i,i+1,0))
	points2D.append((i,i+1))
left_boundary = laneObj.addLeftLaneBoundary(19, True, 0, 1, 1, 0, 2, map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW, points)
right_boundary = laneObj.addRightLaneBoundary(19, True, 0, 1, 1, 0, 2, map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW, points)
central_curve = laneObj.addCentralCurve(10, 1, 1, 1, 0, 1, points)
polygon = laneObj.getPolygon()
laneObj.addLeftRoadSampleAssoc(1.0, 20.0)
laneObj.addRightRoadSampleAssoc(1.0, 20.0)
laneObj.addOverlap(2)	
laneObj.addPredecessor(1)
laneObj.addLeftForwardNeighbor(3)
laneObj.addLeftForwardNeighbor(3)
laneObj.setType(map_lane_pb2.Lane.CITY_DRIVING)
laneObj.setTurn(map_lane_pb2.Lane.NO_TURN)
laneObj.laneSampling(points2D, 3.3, left_boundary, right_boundary, central_curve)
roadObj = Road(1, map)
roadObj.addSection(2)
roadObj.addLanes2Section([1,2,3])
roadObj.addJunction(3)
roadObj.addRoadBoundary(map_road_pb2.BoundaryEdge.NORMAL, 0, 0, 0, 1, 1, 100, points)


