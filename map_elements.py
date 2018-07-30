import numpy as np
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_road_pb2
from modules.map.proto import map_overlap_pb2
from modules.map.proto import map_junction_pb2
from modules.map.proto import map_geometry_pb2


class Overlap:

    def __init__(self, id):
        self._id = id

    def _basic_info_element(self, object, obj_type):
        if obj_type is map_lane_pb2.Lane:
            object.lane_overlap_info.SetInParent()
        elif obj_type is map_junction_pb2.Junction:
            object.junction_overlap_info.SetInParent()

    def _complete_info(self, info, lane, other):
        if type(other) is not map_lane_pb2.Lane:
            info.is_merge = False

    def add(self, map, obj1, obj2):
        apollo_obj1 = obj1.apollo_obj
        apollo_obj2 = obj2.apollo_obj

        # Add overlap object to the map
        overlap = map.overlap.add()
        overlap.id.id = self._id
        # Add the overlapping objects
        object1 = overlap.object.add()
        object1.id.id = obj1._id
        object2 = overlap.object.add()
        object2.id.id = obj2._id
        # Add the specific overlapping objects information
        self._basic_info_element(object1, type(obj1))
        self._basic_info_element(object2, type(obj2))
        # Complete the overlapping objects in case of lanes
        if type(apollo_obj1) is map_lane_pb2.Lane:
            self._complete_info(object1.lane_overlap_info, obj1, obj2)
        if type(apollo_obj2) is map_lane_pb2.Lane:
            self._complete_info(object2.lane_overlap_info, obj2, obj1)

        # Store a reference to the overlap object
        self.apollo_obj = overlap

        # Add the overlap object to the corresponding objects
        obj1_overlap = obj1.overlap_id.add()
        obj1_overlap.id = self._id
        obj2_overlap = obj2.overlap_id.add()
        obj2_overlap.id = self._id


class Junction:

    def __init__(self, id):
        self._id = id

    def add(self, map, junc_poly):
        # Add the junction object to the map
        junction = map.junction.add()
        junction.id.id = self._id
        # Add the polygon vertices
        for vertex in junc_poly:
            point = junction.polygon.point.add()
            point.x = vertex[0]
            point.y = vertex[1]

        # Store a reference to the overlap object
        self.apollo_obj = junction
