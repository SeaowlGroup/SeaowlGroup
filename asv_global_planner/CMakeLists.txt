cmake_minimum_required(VERSION 2.8.3)
project(asv_global_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  asv_simulator
  asv_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
#find_package(Eigen REQUIRED)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   asv_msgs
#   nav_msgs
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES asv_ctrl_vo
 CATKIN_DEPENDS asv_simulator asv_msgs geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(asv_global_planner
  src/asv_global_planner.cpp
)

## Declare a cpp executable
add_executable(asv_global_planner_node src/asv_global_planner_node.cpp src/asv_global_planner.cpp src/asv_a_star.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(asv_global_planner_node asv_global_planner_node asv_global_planner_generate_messages_cpp)
add_dependencies(asv_global_planner_node asv_global_planner_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(asv_global_planner_node
  ${catkin_LIBRARIES}
)
