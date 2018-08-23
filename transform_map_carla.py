import pickle as pkl
import os
from misc import *
from utils import *
from map_elements import *
from modules.map.proto import map_pb2
from modules.map.proto import map_lane_pb2


TOWN = "Town02"

LANES_SEPARATED = True
CROPPED_LANE_SEPARATORS_FILE = 'cropped_lane_separators.pkl'
CONTOURS_GENERATED = True
CONTOURS_FILE = 'contours.pkl'
RAW_DATA_GENERATED = True
RAW_DATA_FILE = 'raw_data.pkl'
FILTER_MAP_POINTS = False

# Constant values
SOLID_YELLOW = map_lane_pb2.LaneBoundaryType.SOLID_YELLOW
CURB = map_lane_pb2.LaneBoundaryType.CURB
DOTTED_WHITE = map_lane_pb2.LaneBoundaryType.DOTTED_WHITE

SPEED_LIMIT = 60
LANE_TYPE = map_lane_pb2.Lane.CITY_DRIVING
HEADING = 0
WIDTH = 3.98
DIRECTION = map_lane_pb2.Lane.FORWARD
SECTION_ID = "1"


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
    map_path = f"maps/{TOWN:s}-colored.png"
    map_img = cv2.imread(map_path)
    lane_map_path = f"maps/{TOWN:s}-lanes-right-left.png"
    lane_map_img = cv2.imread(lane_map_path)
    road_objects_file = f"maps/{TOWN:s}_objects.txt"

    vmaps_dir = 'Carla_Vmaps'
    save_dir = os.path.join(vmaps_dir, TOWN)
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
                contours['lanes'][lntype][i] = list(map(lambda x: np.array([x[0], -x[1]-182]), contours['lanes'][lntype][i]))
        for i in range(len(junctions)):
            junctions[i] = list(map(lambda x: np.array([x[0], -x[1]+182]), junctions[i]))

        # Convert lane points from pixel coordinates to Carla coordinates
        for lntype in contours['lanes']:
            for i in range(len(contours['lanes'][lntype])):
                contours['lanes'][lntype][i] = list(map(pixel_to_coord, contours['lanes'][lntype][i]))

        # Convert junction polygons from pixel coordinates to Carla coordinates
        for i in range(len(junctions)):
            junctions[i] = list(map(pixel_to_coord, junctions[i]))

        # Save raw data
        pkl.dump((contours, junctions), open(RAW_DATA_FILE, 'wb'))

    # Create map object
    map = map_pb2.Map()
    road_dict = {}
    lane_dict = {}
    junction_dict = {}
    overlap_dict = {}
    ID = 1
    all_lanes = contours['lanes']
    # Create road and lane objects
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

            kwargs = {
                'speed_limit': SPEED_LIMIT,
                'lane_turn': LANE_TURN,
                'lane_type': LANE_TYPE,
                'heading': HEADING,
                'width': WIDTH,
                'direction': DIRECTION,
                'do_sampling': FILTER_MAP_POINTS
            }
            new_lane.add(lane, junction_dict, **kwargs)
            lane_dict[str(lane_ID)] = {'lane': new_lane,
                                       'data': lane,
                                       'type': lntype}
            ID += 1

    # Add lane successors and predecessors
    for lane in lane_dict:
        pred_ids = []
        succ_ids = []
        # Detect the lane's predecessor
        for key in lane_dict:
            if np.all(lane_dict[lane]['data'][0] == lane_dict[key]['data'][-1]):
                pred_ids.append(lane_dict[key]['lane'].get_id())
        # Detect the lane's successor
        for key in lane_dict:
            if np.all(lane_dict[lane]['data'][-1] == lane_dict[key]['data'][0]):
                succ_ids.append(lane_dict[key]['lane'].get_id())

        for succ_id in succ_ids:
            lane_dict[lane]['lane'].add_successor(succ_id)
        for pred_id in pred_ids:
            lane_dict[lane]['lane'].add_predecessor(pred_id)

    # Add traffic lights
    traffic_coords = [[[186,10],[191,10],[196,10]],[[186,-73],[189,-73],[196,-73]],
                        [[58,-117],[58,-120],[58,-127]],[[7,-2],[7,-5],[7,-11]],
                        [[29,-12.5],[29,-5],[29,-2.5]],[[119,-14],[119,-5],[119,-2]],
                        [[178,-13],[178,-8],[178,-3]],[[147,-51],[147,-56],[147,-61]],
                        [[140,-21],[135,-21],[130,-21]],[[147,-1],[147,-6],[147,-11]],
                        [[131,-45],[136,-45],[141,-45]],[[59,-2],[59,-7],[59,-12]],
                        [[38,-45],[42,-45],[48,-45]],[[57,-53],[57,-58],[57,-63]],
                        [[122,-62],[122,-57],[122,-52]],[[129,-45],[134,-45],[139,-45]],
                        [[139,-21],[134,-21],[129,-21]]]
    for i in range(len(traffic_coords)):
        s = Signal(i, map)
        types = [map_signal_pb2.Subsignal.CIRCLE, map_signal_pb2.Subsignal.CIRCLE, map_signal_pb2.Subsignal.CIRCLE]
        type = map_signal_pb2.Signal.MIX_3_VERTICAL
        heading = 0
        s.add(3, types, type, traffic_coords[i], heading, [i])

    # Create basic junction objects
    for junction in junctions:
        junc = Junction(str(ID), map)
        junc.add(junction)
        junction_dict[str(ID)] = junc
        ID += 1

    # Create overlap objects
    lanes = []
    for l_id in lane_dict:
        lanes.append(lane_dict[l_id]['lane'])
    junctions = []
    for j_id in junction_dict:
        junctions.append(junction_dict[j_id])

    completed_roads = {}
    for i in range(len(lanes)):
        # Check overlap with other lanes
        for j in range(i + 1, len(lanes)):
            if intersect(lanes[i], lanes[j]):
                overlap_id = "overlap_{}".format(str(ID))
                overlap = Overlap(overlap_id, map)
                overlap.add(lanes[i], lanes[j])
                overlap_dict[overlap_id] = overlap
                ID += 1

        # Check overlap with a junction and set lane boundaries types accordingly
        in_junction = False
        for junction in junctions:
            if intersect(lanes[i], junction):
                in_junction = True
                overlap_id = "overlap_{}".format(str(ID))
                overlap = Overlap(overlap_id, map)
                overlap.add(lanes[i], junction)
                overlap_dict[overlap_id] = overlap
                ID += 1
                # Link junction to road
                road_id = lanes[i].get_id().split('_')[0]
                j_ids = [junction.get_id()]
                l_ids = [lanes[i].get_id()]
                road_dict[road_id].add(lanes[i], SECTION_ID, j_ids, l_ids, HEADING)
                completed_roads[road_id] = 1
                break
        # Set lane boundary types
        if in_junction:
            lanes[i].set_left_lane_boundary_type(DOTTED_WHITE, True)
            lanes[i].set_right_lane_boundary_type(DOTTED_WHITE, True)
        else:
            lanes[i].set_left_lane_boundary_type(SOLID_YELLOW, False)
            lanes[i].set_right_lane_boundary_type(CURB, False)


    # Associate lanes to the roads not treated yet
    for road_id in road_dict:
        if road_id not in completed_roads:
            lane_id = road_id + "_1_-1"
            j_ids = []
            l_ids = [lane_id]
            road_dict[road_id].add(lane_dict[lane_id]['lane'], SECTION_ID, j_ids, l_ids, HEADING)

    # Write base/sim map to file
    if FILTER_MAP_POINTS:
        filename = os.path.join(vmaps_dir, TOWN, 'sim_map.txt')
    else:
        filename = os.path.join(vmaps_dir, TOWN, 'base_map.txt')
    map_file = open(filename, 'w')
    map_file.write(str(map))
    map_file.close()
