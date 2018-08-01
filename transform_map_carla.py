import pickle as pkl
import os
from utils import *
from map_elements import *
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2


LANES_SEPARATED = True
CROPPED_LANE_SEPARATORS_FILE = 'cropped_lane_separators.pkl'
CONTOURS_GENERATED = True
CONTOURS_FILE = 'contours.pkl'
RAW_DATA_GENERATED = False
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

        # Stick lanes in junctions to straight lanes
        stick_lanes(contours['lanes']['left_merging'], contours['lanes']['straight'])
        stick_lanes(contours['lanes']['left_branching'], contours['lanes']['straight'])
        stick_lanes(contours['lanes']['right_merging'], contours['lanes']['straight'])
        stick_lanes(contours['lanes']['right_branching'], contours['lanes']['straight'])

        # Negate y coordinate
        for lntype in contours['lanes']:
            for i in range(len(contours['lanes'][lntype])):
                contours['lanes'][lntype][i] = list(map(lambda x: np.array([x[0], -x[1]]), contours['lanes'][lntype][i]))

        # Convert junction polygons from pixel coordinates to Carla coordinates
        for i in range(len(junctions)):
            junctions[i] = list(map(lambda x: np.array([x[0], -x[1]]), junctions[i]))

        # Convert lane points from pixel coordinates to Carla coordinates
        # for lntype in contours['lanes']:
        #     for i in range(len(contours['lanes'][lntype])):
        #         contours['lanes'][lntype][i] = list(map(pixel_to_coord, contours['lanes'][lntype][i]))
        #
        # # Convert junction polygons from pixel coordinates to Carla coordinates
        # for i in range(len(junctions)):
        #     junctions[i] = list(map(pixel_to_coord, junctions[i]))

        # Save raw data
        pkl.dump((contours, junctions), open(RAW_DATA_FILE, 'wb'))

    # Create map object
    map = map_pb2.Map()

    # Constant values
    SPEED_LIMIT = 60
    LANE_TYPE = map_lane_pb2.Lane.CITY_DRIVING
    LANE_BOUNDARY_LEFT = map_lane_pb2.LaneBoundaryType.SOLID_YELLOW
    LANE_BOUNDARY_RIGHT = map_lane_pb2.LaneBoundaryType.CURB
    HEADING = 0
    WIDTH = 3.98
    SECTION_ID = "1"

    road_dict = {}
    lane_dict = {}
    junction_dict = {}
    overlap_dict = {}
    ID = 1
    all_lanes = contours['lanes']
    for lntype in all_lanes:
        for lane in all_lanes[lntype]:
            # Create basic road object
            road_dict[str(ID)] = Road(str(ID), map)
            # Create basic lane object
            lane_ID = str(ID) + '_1_-1'
            new_lane = Lane(lane_ID, map)
            LANE_TURN = {'straight': map_lane_pb2.Lane.NO_TURN,
                         'left_merging': map_lane_pb2.Lane.LEFT_TURN,
                         'left_branching': map_lane_pb2.Lane.LEFT_TURN,
                         'right_merging': map_lane_pb2.Lane.RIGHT_TURN,
                         'right_branching': map_lane_pb2.Lane.RIGHT_TURN}[lntype]
            VIRTUAL = (lntype != 'straight')
            kwargs = {
                'speed_limit': SPEED_LIMIT,
                'lane_turn': LANE_TURN,
                'lane_type': LANE_TYPE,
                'heading': HEADING,
                'left_boundary': LANE_BOUNDARY_LEFT,
                'right_boundary': LANE_BOUNDARY_RIGHT,
                'virtual': VIRTUAL,
                'width': WIDTH
            }
            new_lane.add(lane, **kwargs)
            lane_dict[str(lane_ID)] = new_lane
            ID += 1

    # Create basic junction objects
    for junction in junctions:
        junc = Junction(str(ID), map)
        junc.add(junction)
        junction_dict[str(ID)] = junc
        ID += 1

    # Create overlap objects
    potential_overlap = []
    for l_id in lane_dict:
        potential_overlap.append(lane_dict[l_id])
    for j_id in junction_dict:
        potential_overlap.append(junction_dict[j_id])

    completed_roads = {}
    for i in range(0, len(potential_overlap) - 1):
        for j in range(i + 1, len(potential_overlap)):
            if intersect(potential_overlap[i], potential_overlap[j]):
                overlap_id = "overlap_{}".format(str(ID))
                overlap = Overlap(overlap_id, map)
                overlap.add(potential_overlap[i], potential_overlap[i + 1])
                overlap_dict[overlap_id] = overlap
                ID += 1

                # Link junctions to roads in case the overlap is between a junction and a lane
                if type(potential_overlap[i]) == Lane:
                    if type(potential_overlap[j]) == Junction:
                        road_id = potential_overlap[i].get_id().split('_')[0]
                        j_ids = [potential_overlap[j].get_id()]
                        l_ids = [potential_overlap[i].get_id()]
                        road_dict[road_id].add(potential_overlap[i], SECTION_ID, j_ids, l_ids, HEADING)
                        completed_roads[road_id] = 1
                if type(potential_overlap[i]) == Junction:
                    if type(potential_overlap[j]) == Lane:
                        road_id = potential_overlap[j].get_id().split('_')[0]
                        j_ids = [potential_overlap[i].get_id()]
                        l_ids = [potential_overlap[j].get_id()]
                        road_dict[road_id].add(potential_overlap[j], SECTION_ID, j_ids, l_ids, HEADING)
                        completed_roads[road_id] = 1

    # Associate lanes to the roads not treated yet
    for road_id in road_dict:
        if road_id not in completed_roads:
            lane_id = road_id + "_1_-1"
            j_ids = []
            l_ids = [lane_id]
            road_dict[road_id].add(lane_dict[lane_id], SECTION_ID, j_ids, l_ids, HEADING)

    # Write base map to file
    map_file = open('base_map.txt', 'w')
    map_file.write(str(map))
    map_file.close()
