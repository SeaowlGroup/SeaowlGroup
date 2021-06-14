cmake_minimum_required(VERSION 2.8.3)
project(asv_map_processing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

#generate_messages(
#  DEPENDENCIES
#  nav_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs roscpp
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(asv_map_processing_node src/asv_map_processing_node.cpp)

add_dependencies(asv_map_processing_node asv_map_processing_node)

target_link_libraries(asv_map_processing_node
  ${catkin_LIBRARIES}
)
