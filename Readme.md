# Intro

The purpose of this app is to obtain HDMaps in OpenDrive format, required by the Apollo autonomous driving framework, for towns in the Carla driving simulator, using images of the town maps in a format that will be described below.

# Requirements

Note: We will use RGB notation when reffering to colors.
In order to generate an HDMap for a town named TOWN, we first need the following types of map images:
- TOWN-colored.png - an image with lanes colored in blue (5,5,255) and green (5,255,0), for opposite driving directions, and junctions colored in white (255,255,255). The junctions cover just one lane on the part where the lanes are going straight. Also, the stright lines in the junction's polygon need to be smooth, keeping the x or y coordinate constant along the straight portion.
- TOWN-lanes-right-left.png - additionally to the image above, this image also contains lines for the center of the lanes. The color code for the centers of the lanes is the following:
    -  straight: (255,0,0)
    -  left merging: (255,200,0)
    -  left branching: (255,150,0)
    -  right merging: (255,50,0)
    -  right branching: (255,100,0)

The proto modules for generating OpenDrive map elements are also needed, so make sure to link the APOLLO_DIR/py_proto/modules/map/proto directory to your project. In case you are running the project in Pycharm, follow the steps File->Settings->Project Interpreter->select "Show All" in the "Project Interpreter" dropdown->select "Show paths" in the Project Interpreter menu and add the desired path.

# Get the HDMap

You have to run **transform_map_carla.py** to obtain the HDMap. In the script, set field TOWN to the desired town name.
In order to reduce the time of processing, intermediary files are generated with the lane separators, the basic contours (road edges, lane separators and lane centers) and the final contours and junctions' polygons. To permit generation of these intermediary files, set the LANES_SEPARATED, CONTOURS_GENERATED or RAW_DATA_GENERATED fields to True.
Dreamview needs a **base_map.txt** file and a **sim_map.txt** file, containing filtered information from the base_map file. In order to generate base_map.txt, set the FILTER_MAP_POINTS field to False, and to generate the sim_map.txt file, set it to True.

# Run Dreamview with your HDMap

Create a folder MAP_DIR=APOLLO_DIR/modules/map/data/CARLA_TOWN and run the commands:

```sh
cd APOLLO_DIR
./scripts/generate_routing_topo_graph.sh --map_dir MAP_DIR
```

Then add the line

```sh
--map_dir MAP_DIR
```

to the modules/common/data/global_flagfile.txt file.

Run

```sh
./scripts/bootstrap.sh
```

and you are ready to use Apollo! 