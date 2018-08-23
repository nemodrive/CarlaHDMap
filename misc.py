import sys
import numpy as np
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_stop_sign_pb2
from modules.map.proto import map_speed_bump_pb2
from modules.map.proto import map_crosswalk_pb2
from modules.map.proto import map_parking_space_pb2
from modules.map.proto import map_signal_pb2
from shapely.geometry import LineString, Point
import math

class StopSign:
    def __init__(self, id, map):
        self.stop_sign = map.stop_sign.add()
        self.stop_sign.id.id = str(id)
        self._id = id
    
    def add(self, points, heading, type, overlap_ids):
        stop_line = self.stop_sign.stop_line.add()
        segment = stop_line.segment.add()
        segment.start_position.x = points[0][0]
        segment.start_position.y = points[0][1]
        prevX, prevY = points[0]
        length = 0
        for point in points:
            p = segment.line_segment.point.add()
            p.x, p.y = point
            length += math.sqrt(math.pow(p.x - prevX, 2) + math.pow(p.y - prevY, 2))
            prevX = p.x
            prevY = p.y
        segment.length = length
        segment.heading = heading
        for o in overlap_ids:
            self.stop_sign.overlap_id.add().id = str(o)
        self.stop_sign.type = type
    def getID(self):
        return self._id


#testing stop_sign
# map = map_pb2.Map()
# s = StopSign(1, map)
# points2D = []
# for i in range(10):
#     points2D.append((i,i+1))
# s.add(points2D, 1, map_stop_sign_pb2.StopSign.ONE_WAY, [1,2,3])

class Crosswalk:
    def __init__(self, id, map):
        self.crosswalk = map.crosswalk.add()
        self.crosswalk.id.id = str(id)
        self._id = id

    def add(self, overlap_ids, points):
        for id in overlap_ids:
            self.crosswalk.overlap_id.add().id = str(id)
        
        for p in points:
            point = self.crosswalk.polygon.point.add()
            point = p
        
    def getID(self):
        return self._id

#testing crosswalk
# c = Crosswalk(1, map)
# c.add([1,2,3], points2D)

class SpeedBump:
    def __init__(self, id, map):
        self.speed_bump = map.speed_bump.add()
        self.speed_bump.id.id = str(id)
        self._id = id
        
    def add(self, points, heading, overlap_ids):
        for id in overlap_ids:
            self.speed_bump.overlap_id.add().id = str(id)
    
        position = self.speed_bump.position.add()
        segment = position.segment.add()
        segment.start_position.x = points[0][0]
        segment.start_position.y = points[0][1]
        prevX, prevY = points[0]
        length = 0
        for point in points:
            p = segment.line_segment.point.add()
            p.x, p.y = point
            length += math.sqrt(math.pow(p.x - prevX, 2) + math.pow(p.y - prevY, 2))
            prevX = p.x
            prevY = p.y
        segment.length = length
        segment.heading = heading

    def getID(self):
        return self._id

#testing speed_bump
# sb = SpeedBump(1, map)
# sb.add(points2D, 1, [1,2,3])

class ParkingSpace:
    def __init__(self, id, map):
        self.parking_space = map.parking_space.add()
        self.parking_space.id.id = str(id)
        self._id = id
        
    def add(self, overlap_ids, heading, points):
        for id in overlap_ids:
            self.parking_space.overlap_id.add().id = str(id)
        
        for point in points:
            p = self.parking_space.polygon.point.add()
            p = point
        
        self.parking_space.heading = heading

#testing parking_space
# ps = ParkingSpace(1, map)
# ps.add([1,2,3], 0, points2D)

class Signal:
    def __init__(self, id, map):
        self.signal = map.signal.add()
        self.signal.id.id = str(id)
        self._id = id
        
    def add(self, subsignal_nr, types, type, points, heading, overlap_ids):
        heights = [-25.0508544, -24.63178106, -24.21270773]
        for i in range(len(points)):
            p = self.signal.boundary.point.add()
            p.x = points[i][0]
            p.y = points[i][1]
            p.z = heights[0]
        subsignal = []
        for i in range(subsignal_nr):
            s = self.signal.subsignal.add()
            subsignal.append(s)
            subsignal[i].id.id = str(i)
            subsignal[i].type = types[i]
            subsignal[i].location.x = points[0][0]
            subsignal[i].location.y = points[0][1]
            subsignal[i].location.z = heights[i%3]
            
        for i in overlap_ids:
            self.signal.overlap_id.add().id = str(i)
            
        self.signal.type = type
        stop_line = self.signal.stop_line.add()
        
        segment = stop_line.segment.add()
        segment.start_position.x = points[0][0]
        segment.start_position.y = points[0][1]
        prevX, prevY = points[0]
        length = 0
        for point in points:
            p = segment.line_segment.point.add()
            p.x, p.y = point
            length += math.sqrt(math.pow(p.x - prevX, 2) + math.pow(p.y - prevY, 2))
            prevX = p.x
            prevY = p.y
        segment.length = length
        segment.heading = heading
        
#testing signal
# s = Signal(1, map)
# point = [0,0]
# types = [map_signal_pb2.Subsignal.CIRCLE, map_signal_pb2.Subsignal.CIRCLE, map_signal_pb2.Subsignal.CIRCLE]
# type = map_signal_pb2.Signal.MIX_3_VERTICAL
# heading  = 0
# s.add(point, 3, types, [1,2,3],  type, points2D, heading)

