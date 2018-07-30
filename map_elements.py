import cv2
import numpy as np
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_road_pb2
from modules.map.proto import map_overlap_pb2
from modules.map.proto import map_junction_pb2
from modules.map.proto import map_geometry_pb2
from utils import distance
from structures import *


class Overlap:

    def __init__(self, id, map):
        self._id = id
        self.overlap = map.overlap.add()
        self.overlap.id.id = id

    def getID(self):
        return self._id

    def _basic_info_element(self, object, obj_type):
        if obj_type is Lane:
            object.lane_overlap_info.SetInParent()
        elif obj_type is Junction:
            object.junction_overlap_info.SetInParent()

    def _complete_info(self, info, lane, other):
        # Find end points of overlap
        polygon = other.getPolygon()
        central_curve = lane.lane.central_curve.segment[0].line_segment.point
        length = 0
        prev_point = np.array(central_curve[0].x, central_curve[0].y)
        start_s = 0
        end_s = 0
        i = 0
        while cv2.pointPolygonTest(polygon, prev_point, False) < 0 and i < len(central_curve):
            curr_point = np.array(central_curve[i].x, central_curve[i].y)
            length += distance(prev_point, curr_point)
            prev_point = curr_point
        if i < len(central_curve):
            start_s = length
            while cv2.pointPolygonTest(polygon, prev_point, False) >= 0 and i < len(central_curve):
                curr_point = np.array(central_curve[i].x, central_curve[i].y)
                length += distance(prev_point, curr_point)
                prev_point = curr_point
            end_s = length

        info.start_s = start_s
        info.end_s = end_s

        # Check if the lane is merging with the other object
        length = 0
        LENGTH_THRESH = 1.0
        in_other_poly = False
        i = 0
        prev_point = np.array(central_curve[0].x, central_curve[0].y)
        while length < LENGTH_THRESH and i < len(central_curve):
            curr_point = np.array(central_curve[i].x, central_curve[i].y)
            length += distance(prev_point, curr_point)
            prev_point = curr_point
            if cv2.pointPolygonTest(other.polygon, prev_point, False) >= 0:
                in_other_poly = True
                break

        if type(other) is Lane and in_other_poly:
            info.is_merge = True
        else:
            info.is_merge = False


    def add(self, obj1, obj2):
        # Add the overlapping objects
        object1 = self.overlap.object.add()
        object1.id.id = obj1.getID()
        object2 = self.overlap.object.add()
        object2.id.id = obj2.getID()
        # Add the specific overlapping objects information
        self._basic_info_element(object1, type(obj1))
        self._basic_info_element(object2, type(obj2))
        # Complete the overlapping objects in case of lanes
        if type(obj1) is Lane:
            self._complete_info(object1.lane_overlap_info, obj1, obj2)
        if type(obj2) is Lane:
            self._complete_info(object2.lane_overlap_info, obj2, obj1)

        # Add the overlap object to the corresponding objects
        obj1_overlap = obj1.overlap_id.add()
        obj1_overlap.id = self._id
        obj2_overlap = obj2.overlap_id.add()
        obj2_overlap.id = self._id


class Junction:

    def __init__(self, id, map):
        self._id = id
        self.junction = map.junction.add()
        self.junction.id.id = id

    def getID(self):
        return self._id

    def add(self, junc_poly):
        self._polygon = junc_poly

        # Add the polygon vertices
        for vertex in junc_poly:
            point = self.junction.polygon.point.add()
            point.x = vertex[0]
            point.y = vertex[1]

    def getPolygon(self):
        return self._polygon

