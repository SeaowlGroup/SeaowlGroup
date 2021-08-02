# The Planner Tester Package
## Installation
This package was coded on Ubuntu 20.04 LTS with ROS Noetic. It wouldn't work on previous versions of ROS because it uses Python 3.
Here are the steps to take to install it from scratch, starting from a plain Ubuntu 20.04 distribution.
### ROS Installation
The first step is to enable access to the repositories universe, multiverse and restricted if it not already. This needs to be typed in the terminal :
```
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
```
Then, we need to add the key to the ROS distribution :
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
Before launching the installation, check that all your packages are up to date :
```
sudo apt-get update && sudo apt-get upgrade
```
Finally, launch the installation (here we used the full version) :
```
sudo apt install ros-noetic-desktop-full
```
Before continuing, don't forget to source the ROS setup (**this needs to be done each time a new terminal is opened**) :
```
source /opt/ros/noetic/setup.bash
```
### Package Installation
Before installing the main package, we need to install the additional depedencies (some more may be necessary depending on your configuration) :
```
sudo apt install ros-noetic-tf2
sudo apt install libeigen3-dev
sudo apt install python3-pip
pip3 install pandas
pip3 install openpyxl
pip3 install scipy
pip3 install datetime
```
Finally, you need to be located in the directory where you want to install your ROS workspace, and proceed :
```
mkdir -p workspace/src/
cd workspace/src/
git clone https://github.com/Straccia11/seaowl.git
cd ..
catkin_make
```
(**Some problems may occur because of the package asv_msgs**)

You will need to source the package before starting to use it and **everytime you open a new terminal**. From the location of your workspace :
```
source devel/setup.bash
```
## Use

### Get Started
To launch a graphic user interface allowing to set easily the parameters, type in the terminal :
```
roscd asv_system
python3 executable5.py
```
### Launch Simulations
There are two ways of properly using the package : with launchfiles or with python APIs.

- Launchfiles are located in `asv_system/launch` and can be launched with
```
roslaunch asv_system name_of_the_launchfile.launch
```
- APIs are the python files located in `asv_system` and need to be executed by Python 3
```
roscd asv_system
python3 executable_file.py
```

The parameters can either be set manually in the launch files if the simulation is launched that way, either be entered in a graphic interface for `executable5.py`, or be set in an Excel or YAML file put in `asv_system/param/` and executed respectively with `executable6.py` or `executable9.py`.

### Output
When a simulation is over, an input file and an output file (both plain text files) are respectively created in `asv_system/input/` and `asv_system/output/`, named after a serial number concatenating the year, month, day, hour, minute and second the simulation was launched. These informations can be plotted using `asv_system/graph_drawer.py`


## Content

### Launch Files and Executables

***

### Packages
This package contains:
+ `asv_ctrl_vo`: an implementation of the "Velocity Obstacle" algorithm for
collision avoidance
+ `asv_global_planner`: an implementation of the "A Star" algorithm for
collision avoidance
+ `asv_map_processing`: a package meant for implementing the inflation of static obstacles
+ `asv_msgs`: message types used in the system
+ `asv_obstacle_tracker`: package that acts as a "black box", providing
information about the states (and possibly metadata) that a collision avoidance
system can subscribe to. _It does not actually track obstacles._ It is also
possible to simulate the addition of sensor noise using this package. It also contains a node that directly simulate the obstacles if their trajectory is straight
+ `asv_path_trackers`: implements the (Integral) Line of Sight (LOS) method and
a simple pure pursuit scheme for path following
+ `asv_referee`: package that contains all the node that contributes to the coordination and evaluation of the behavior of the ASV
+ `asv_simulator`: simulates a nonlinear 3DOF surface vessel
+ `state_estimator`: unfinished package for estimating the ASV pose given GPS
and IMU data
+ `asv_system`: metapackage with launch files and more

### Nodes

| Package | Node(s) |
| --- | --- |
| `asv_referee` | `referee_node`, `reaper_node` |
| `asv_path_trackers` | `LOS_node`, `PP_node` |
| `asv_obstacle_trackers` | `obstacle_tracker_node`  |
| `asv_simulator` | `simulator_node` |
| `asv_global_planner` | `global_planner_node` |
| `asv_ctrl_vo` | `ctrl_vo_node` |
| `asv_map_processing` | `map_processing_node` |

#### Main Nodes (required)
+ `simulator_node`: simulates the vessel
+ `LOS_node` / `PP_node`: implement a path tracker algorithm (either _Line Of Sight_ or _Pure Pursuit_) over an array of waypoints (the difference between the two algorithms can be found in  [Thomas Stenersen's thesis](https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2352498/12747_FULLTEXT.pdf?sequence=1&isAllowed=y))
+ `obstacle_tracker_node` / `obstacle_simplified_node`: transmits the positions and velocities of the other ships to the ASV, _`obstacle_simplified_node`_ also simulates the obstacles if their trajectories are straight and their velocities constant (otherwise _`obstacle_tracker_node`_ needs them to be independantly simulated with a _`simulator_node`_ for each)

#### Additional Important Nodes
+ `referee_node`: calculates all the performance indicators and etablish the times of beginning and end of the simulation. Can be set to _required_ in the launch file to automatically close the program at the end of the simulation (only for single processing)
+ `reaper_node`: in case of multiprocessing, kills all the nodes once they are all finished. Requires _`referee_node`_
+ `global_planner_node`: implements the global_planner, the one included is an _A Star_ planner but an other one can be integrated easily
+ `ctrl_vo_node`: implements the local planner in this case a _Velocity Obstacle_ planner. Another one can be integrated, but it would need to change the file architecture a bit
+ `map_processing_node`: if the simulation contains static obstacles (basically a map), creates a security margin with them by implementing obstacle inflation. Requires the _`map_server`_ node, which is included in the default ROS packages


#### Deprecated Nodes (for information)
+ `clock_node`: meant to accerate the simulation thus shortening its duration. Caused a degradation of the behavior of the algorithms and was then left out
+ `state_simulator_node`: incomplete implementation of the estimation of the ASV pose via GPS
+ `obstacle_tracker_nema_node`: can simulate and track the obstacles simulated by the emission of AIS signals via an UDP port. Was meant to be used with the software _Nema Studio_ but this possibility was left out

### Main Topics
_The **Architecture** part describes where each node publish and subscribe._

The topics are often included in diverse namespaces indicating the opus and/or vessel related to the specific topic.

- `/state` (type ): real pose and twist of a vessel
- `/LOS/cmd_vel / /PP/cmd_vel`, `/cmd_vel` (type ): twist of a vesset respectively calculated by the path tracker and the local planner  
- `/obstacle_states` (type StateArray):
- `/end_simulation`, `/start_simulation` (type _Empty_): when a message is published, signal respectively the end or the beginning of the simulation (used to synchronize the nodes and automate the execution of the successive opuses)
- `/map`, `/processed_map`, `/localmap` (optionnal, type _Occupancy Grid_): if a static map is set in the parameters, `/map` is its conversion into an occupancy grid, `/processed_map` is the implementation of the inflated static obstacles and `/local_map` is a short-ranged non-static version of the map used by the local planner  




### Additional Message Types

All the message types specific to this package are detailed in the sub-package`/asv_msgs`.

### Utility Scripts

- `clear.sh`: removes the input and output or either a specific simulation or all the scripts
- `current_opus.sh`: indicates the maximum opus of the latest simulation  
- `kill.sh`: kills all processes related to the package (to use when a simulation or several are running in the background)               
- `concatenate.sh`: (**NOT YET WRITTEN**) concatenates two output and remove all the duplicate opuses  
- `graph_drawer.py`: opens a graphical interface allowing to plot data from the results of a specific simulation
- `launch_background.sh`: launches `executable9.py` / `executable9Adrien.py` in the backgound and write the output and errors in the directory `log/` (**the parameters are in executable9**)
- `watch_cpu.py`: gives information about the CPUs and memory activity over a certain periiod of time (meant to determine how many processes can be launched simultaneously)
- `config.sh`: configures a new machine to be able to use the package (**not meant to be executed as a whole script but line by line**)      
- `is_running.sh`: indicates if the process is still running (**not very trustworthy**)    
- `rename.sh`: renames the input and output of a specific simulation (by default the latest)


## Architecture
### Nodes and Topics

Here is the node graph of a case where there is a local planner but no global planner and `obstacle_simplified_node` is used :
![Graph 1](asv_system/rosgraph.png)
***
Here is another where there is a map, a local planner, a global planner and an obstacle ship simulated with `asv_simulator_node` :
![Graph 2](asv_system/rosgraph2.png)
***
### File architecture
```
.
├── asv_ctrl_vo
│   ├── CMakeLists.txt
│   ├── include
│   │   └── asv_ctrl_vo
│   │       ├── asv_ctrl_vo.h
│   │       └── asv_ctrl_vo_node.h
│   ├── package.xml
│   └── src
│       ├── asv_ctrl_vo.cpp
│       └── asv_ctrl_vo_node.cpp
├── asv_global_planner
│   ├── CMakeLists.txt
│   ├── include
│   │   └── asv_global_planner
│   │       ├── asv_a_star.h
│   │       ├── asv_global_planner.h
│   │       └── asv_global_planner_node.h
│   ├── package.xml
│   └── src
│       ├── asv_a_star.cpp
│       ├── asv_global_planner.cpp
│       └── asv_global_planner_node.cpp
├── asv_map_processing
│   ├── CMakeLists.txt
│   ├── include
│   │   └── asv_map_processing
│   │       └── asv_map_processing_node.h
│   ├── package.xml
│   └── src
│       └── asv_map_processing_node.cpp
├── asv_msgs
│   ├── CMakeLists.txt
│   ├── msg
│   │   ├── Path.msg
│   │   ├── ShipMetaData.msg
│   │   ├── StateArray.msg
│   │   ├── State.msg
│   │   └── Waypoint2D.msg
│   └── package.xml
├── asv_obstacle_tracker
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── default.launch
│   │   ├── nema.launch
│   │   ├── obst_simplified2.launch
│   │   ├── obst_simplified.launch
│   │   └── simple.launch
│   ├── nodes
│   │   ├── crossLaneObstNode.py
│   │   ├── obstacles_simplified_node.py
│   │   ├── obstacle_tracker_nema_node.py
│   │   └── obstacle_tracker_node.py
│   └── package.xml
├── asv_path_trackers
│   ├── CMakeLists.txt
│   ├── nodes
│   │   ├── asv_ctrl_los_node_obstacles.py
│   │   ├── asv_ctrl_los_node.py
│   │   ├── asv_ctrl_pp_node.py
│   │   ├── __pycache__
│   │   │   └── utils.cpython-38.pyc
│   │   ├── utils.py
│   │   └── utils.pyc
│   └── package.xml
├── asv_referee
│   ├── CMakeLists.txt
│   ├── nodes
│   │   ├── asv_clock_node.py
│   │   ├── asv_reaper_node.py
│   │   ├── asv_referee_node2.py
│   │   └── asv_referee_node.py
│   └── package.xml
├── asv_simulator
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── models
│   │   │   ├── revolt.urdf
│   │   │   ├── ship1.urdf
│   │   │   ├── ship2.urdf
│   │   │   ├── simple.urdf
│   │   │   └── viknes.urdf
│   │   ├── parameters
│   │   │   ├── viknes2.yaml
│   │   │   └── viknes.yaml
│   │   ├── rosdoc.yaml
│   │   ├── rviz
│   │   │   ├── config.rviz
│   │   │   ├── exec9.rviz
│   │   │   ├── map_and_proc_map.rviz
│   │   │   ├── one_vessel_with_map.rviz
│   │   │   ├── three_vessels.rviz
│   │   │   ├── toulon.rviz
│   │   │   └── two_vessels.rviz
│   │   └── waypoints
│   │       ├── asv_head_on_and_crossing.yaml
│   │       ├── asv_overtaking_and_crossing.yaml
│   │       ├── asv_overtaking.yaml
│   │       ├── asv_waypoint_list.yaml
│   │       ├── clockwise_rectangle.yaml
│   │       ├── east_to_west.yaml
│   │       ├── in_between_rules.yaml
│   │       ├── left_to_right.yaml
│   │       ├── mozambique.yaml
│   │       ├── north_to_south.yaml
│   │       ├── overtaking_headon_crossing_asv.yaml
│   │       ├── overtaking_headon_crossing_ship1.yaml
│   │       ├── overtaking_headon_crossing_ship2.yaml
│   │       ├── ship1_head_on_and_crossing.yaml
│   │       ├── ship1_overtaking_and_crossing.yaml
│   │       ├── ship1_overtaking.yaml
│   │       ├── ship1_waypoint_list.yaml
│   │       ├── ship2_head_on_and_crossing.yaml
│   │       ├── ship2_overtaking_and_crossing.yaml
│   │       ├── ship2_waypoint_list.yaml
│   │       ├── slow_south_to_north.yaml
│   │       ├── south_to_north.yaml
│   │       ├── test2.yaml
│   │       ├── test.yaml
│   │       ├── waypoint_test_list.yaml
│   │       └── west_to_east.yaml
│   ├── include
│   │   ├── asv_simulator.h
│   │   ├── asv_simulator_node.h
│   │   └── wave_filter.h
│   ├── launch
│   │   ├── default.launch
│   │   ├── test2.launch
│   │   ├── test.launch
│   │   └── test_obst.launch
│   ├── LICENSE
│   ├── mainpage.dox
│   ├── meshes
│   │   ├── boat2.blend
│   │   ├── boat2.dae
│   │   ├── boat2.stl
│   │   ├── boat3.stl
│   │   ├── boat.dae
│   │   ├── boat.stl
│   │   ├── hovik.STL
│   │   ├── Revolt.STL
│   │   ├── testmap.stl
│   │   └── viknes.STL
│   ├── nodes
│   │   ├── data_publisher.py
│   │   ├── fake_asv.py
│   │   ├── meshpublisher.py
│   │   ├── rlog011.csv
│   │   ├── teleop_joy.py
│   │   ├── utils.py
│   │   └── vessel.py
│   ├── package.xml
│   ├── README.md
│   └── src
│       ├── asv_simulator.cpp
│       ├── asv_simulator_node.cpp
│       └── wave_filter.cpp
├── asv_state_estimator
│   ├── CMakeLists.txt
│   ├── nodes
│   │   ├── asv_state_estimator.py
│   │   └── convert_stuff.py
│   └── package.xml
├── asv_system
│   ├── bato.ico
│   ├── bato.png
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── maps
│   │   │   ├── big_block.png
│   │   │   ├── big_block.yaml
│   │   │   ├── carte_-anadarko_uk_v4resize.webp
│   │   │   ├── center_dot.png
│   │   │   ├── center_dot.yaml
│   │   │   ├── chenal_toulon.png
│   │   │   ├── chenal_toulon.yaml
│   │   │   ├── hovik.png
│   │   │   ├── hovik.yaml
│   │   │   ├── island.png
│   │   │   ├── island.yaml
│   │   │   ├── map1.png
│   │   │   ├── map1.yaml
│   │   │   ├── mozambique.png
│   │   │   ├── mozambique.yaml
│   │   │   ├── petit_chenal.png
│   │   │   ├── rade_toulon.png
│   │   │   ├── rade_toulon.yaml
│   │   │   ├── test_color_map.png
│   │   │   ├── test_color_map.yaml
│   │   │   ├── test.png
│   │   │   └── test.yaml
│   │   ├── param
│   │   │   ├── param1.yaml
│   │   │   ├── param2.xlsx
│   │   │   ├── param2.yaml
│   │   │   ├── param3.xlsx
│   │   │   ├── param3.yaml
│   │   │   ├── param4.xlsx
│   │   │   ├── param4.yaml
│   │   │   └── param.xlsx
│   │   └── param3.yaml
│   ├── debug.py
│   ├── debug.txt
│   ├── étalon.py
│   ├── executable1.py
│   ├── executable2.py
│   ├── executable3.py
│   ├── executable4.py
│   ├── executable5.py
│   ├── executable6.py
│   ├── executable8.py
│   ├── executable9Adrien.py
│   ├── executable9.py
│   ├── graph_drawer.py
│   ├── icon.png
│   ├── input
│   │   ├── 210801230453.txt
│   │   ├── first_373.txt
│   │   ├── left_and_rosace_vo1.txt
│   │   ├── left_and_rosace_vo2.txt
│   │   ├── rattrape_rattrapant.txt
│   │   ├── rosace_rattrape.txt
│   │   ├── survivor2.txt
│   │   ├── survivor3.txt
│   │   ├── survivor4.txt
│   │   ├── survivor5.txt
│   │   ├── survivor6.txt
│   │   ├── survivor7.txt
│   │   └── survivor.txt
│   ├── launch
│   │   ├── chenal.launch
│   │   ├── chenal_toulon.launch
│   │   ├── default.launch
│   │   ├── main_launch2.launch
│   │   ├── main_launch3.launch
│   │   ├── main_launch.launch
│   │   ├── mapserver.launch
│   │   ├── mozambique_fast.launch
│   │   ├── mozambique.launch
│   │   ├── mozambique_nema.launch
│   │   ├── mozambique_simplified.launch
│   │   ├── new_moz.launch
│   │   ├── obstacles2.launch
│   │   ├── obstacles.launch
│   │   ├── play_bagfile.launch
│   │   ├── reaper.launch
│   │   ├── test2.launch
│   │   ├── tester.launch
│   │   ├── test.launch
│   │   ├── test_static_obstacle.launch
│   │   └── toulon.launch
│   ├── launch_backgroundAdrien.sh
│   ├── log
│   │   └── nohup.err
│   ├── output
│   │   ├── 210801230453.txt
│   │   ├── dataAdrien.txt
│   │   ├── first_373.txt
│   │   ├── left_and_rosace_vo1.txt
│   │   ├── left_and_rosace_vo2.txt
│   │   ├── rattrape_rattrapant.txt
│   │   ├── rosace_rattrape.txt
│   │   ├── survivor2.txt
│   │   ├── survivor3.txt
│   │   ├── survivor4.txt
│   │   ├── survivor5.txt
│   │   ├── survivor6.txt
│   │   ├── survivor7.txt
│   │   └── survivor.txt
│   ├── output.xlsx
│   ├── package.xml
│   ├── scripts
│   │   ├── clear.sh
│   │   ├── concatenate.py
│   │   ├── config.sh
│   │   ├── current_opus.sh
│   │   ├── graph_drawer.py
│   │   ├── is_running.sh
│   │   ├── kill.sh
│   │   ├── launch_background.sh
│   │   ├── rename.sh
│   │   └── watch_cpu.py
│   ├── Seagull-USV.png
│   ├── simu.xlsx
│   ├── survivor.xls
│   └── test.py
├── cross_lane
│   ├── api
│   │   ├── crossLaneExec.py
│   │   └── crossLaneFile.py
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── param
│   │   │   └── crossLane.yaml
│   │   └── rviz
│   │       └── crossLane.rviz
│   ├── launch
│   │   └── crossLane.launch
│   └── package.xml
├── original_readme.md
└── README.md
```

## Issues and improvements to be made
