import pickle as pkl
import os
import cv2
import pandas as pd
import numpy as np
from utils import *
from map_elements import *
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2
from modules.map.proto import map_road_pb2
from modules.map.proto import map_overlap_pb2
from modules.map.proto import map_junction_pb2


LANES_SEPARATED = True
CROPPED_LANE_SEPARATORS_FILE = 'cropped_lane_separators.pkl'
CONTOURS_GENERATED = True
CONTOURS_FILE = 'contours.pkl'
RAW_DATA_GENERATED = True
RAW_DATA_FILE = 'raw_data.pkl'


def get_points(map_img, lane_map_img):
    # Detect the road edges
    ROAD_EDGE_LOW_THRESH = [0, 0, 1]
    ROAD_EDGE_HIGH_THRESH = [255, 255, 255]
    road_edges = get_edges(map_img, ROAD_EDGE_LOW_THRESH, ROAD_EDGE_HIGH_THRESH)

    points = []
    PID = 1
    for edge in road_edges:
        for point in edge:
            i = point[0]
            j = point[1]
            relative_loc = Density * np.array([i, j])
            coord = relative_loc + MAP_OFFSET

            lat, long = transform_coord_to_lat_long(coord)

            points.append([i, j, PID, lat, long])
            PID += 1
    df_points = pd.DataFrame(points, columns=POINTS_COLUMNS)

    # Detect lane separators
    if LANES_SEPARATED and os.path.exists(CROPPED_LANE_SEPARATORS_FILE):
        lane_separators = pkl.load(open(CROPPED_LANE_SEPARATORS_FILE, 'rb'))
    else:
        lane_separators = get_lane_separators(map_img, df_points)
        pkl.dump(lane_separators, open(CROPPED_LANE_SEPARATORS_FILE, 'wb'))

    for lane in lane_separators:
        for point in lane:
            i = point[0]
            j = point[1]
            relative_loc = Density * np.array([i, j])
            coord = relative_loc + MAP_OFFSET

            lat, long = transform_coord_to_lat_long(coord)

            points.append([i, j, PID, lat, long])
            PID += 1

    # Generate lames
    lanes = read_lane_points(lane_map_img, points)

    df_points = pd.DataFrame(points, columns=POINTS_COLUMNS)

    return df_points, road_edges, lane_separators, lanes


if __name__ == '__main__':
    # Get map image
    town = "Town02"
    map_path = f"maps/{town:s}-colored.png"
    map_img = cv2.imread(map_path)
    lane_map_path = f"maps/{town:s}-lanes-right-left.png"
    lane_map_img = cv2.imread(lane_map_path)
    road_objects_file = f"maps/{town:s}_objects.txt"

    vmaps_dir = 'Carla_Vmaps'
    save_dir = os.path.join(vmaps_dir, town)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Build road edges, lane separators and lanes
    if CONTOURS_GENERATED and os.path.exists(CONTOURS_FILE):
        contours = pkl.load(open(CONTOURS_FILE, 'rb'))
    else:
        df_points, road_edges, lane_separators, lanes = get_points(map_img, lane_map_img)
        # Save contours
        contours = {}
        contours['road_edges'] = road_edges
        contours['lane_separators'] = lane_separators
        contours['lanes'] = lanes
        pkl.dump(contours, open(CONTOURS_FILE, 'wb'))

    if RAW_DATA_GENERATED and os.path.exists(RAW_DATA_FILE):
        contours, junctions = pkl.load(open(RAW_DATA_FILE, 'rb'))
    else:
        # Determine the road's junction polygons
        junctions = detect_junctions(map_img)

        # Split straight lanes at junctions
        contours['lanes']['straight'] = split_at_junctions(contours['lanes']['straight'], junctions)

        # Convert lane points from pixel coordinates to Carla coordinates
        for lntype in contours['lanes']:
            for i in range(len(contours['lanes'][lntype])):
                contours['lanes'][lntype][i] = list(map(pixel_to_coord, contours['lanes'][lntype][i]))

        # Save raw data
        pkl.dump((contours, junctions), open(RAW_DATA_FILE, 'wb'))

    # all_contours = junctions + contours['road_edges'] + contours['lane_separators']
    # for lane in contours['lanes']:
    #     all_contours += contours['lanes'][lane]
    #
    # display_contours(all_contours, map_img.shape)

    # Create map object
    # map = map_pb2.Map()

    # Create basic road objects
    # Create basic lane objects
    # Create basic junction objects
    # Create overlap objects
    # Link junctions to roads
    # lane1 = Lane("1", map)
    # lane2 = Lane("2", map)
    #
    # polygon = np.array([np.array([0,1]), np.array([1,2]), np.array([2,3])])
    # junction = Junction("3", map)
    # junction.add(polygon)
    #
    # print(str(map))

    file = open("/home/alexm/Desktop/base_map.txt", 'rb')
    map = map_pb2.Map()
    # map.ParseFromString(file.read())
