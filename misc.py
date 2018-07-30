import sys
import numpy as np
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_road_pb2
from modules.map.proto import map_stop_sign_pb2
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
map = map_pb2.Map()
s = StopSign(1, map)
points2D = []
for i in range(10):
    points2D.append((i,i+1))
s.add(points2D, 1, map_stop_sign_pb2.StopSign.ONE_WAY, [1,2,3])

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
c = Crosswalk(1, map)
c.add([1,2,3], points2D)

class SpeedBump:
    def __init__(self, id, map):
        self.speed_bump = map.speed_bump.add()
        self.speed_bump.id.id = str(id)
        self._id = str(id)
        
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
sb = SpeedBump(1, map)
sb.add(points2D, 1, [1,2,3])
