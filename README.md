## Installation
This package was coded on Ubuntu 20.04 LTS with ROS Noetic. It wouldn't work on previous versions of ROS because it uses python3.
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

## Usage

***

## Content

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
+ `LOS_node / PP_node`: implement a path tracker algorithm (either _Line Of Sight_ or _Pure Pursuit_) over an array of waypoints (the difference between the two algorithms can be found in  [Thomas Stenersen's thesis](https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2352498/12747_FULLTEXT.pdf?sequence=1&isAllowed=y))
+ `obstacle_tracker_node / obstacle_simplified_node`: transmits the positions and velocities of the other ships to the ASV, _`obstacle_simplified_node`_ also simulates the obstacles if their trajectories are straight and their velocities constant (otherwise _`obstacle_tracker_node`_ needs them to be independantly simulated with a _`simulator_node`_ for each)

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

### Additional Message Types

***

## Architecture
### File architecture
### Nodes and Topics

***

## Launch Files and Executables

***

## Issues and improvements to be made
