#include "ros/ros.h"
#include <ros/console.h>

#include <iostream>

#include "nav_msgs/OccupancyGrid.h"

#include "asv_global_planner/test_global_planner.h"


ExerciseGlobalPlanner::ExerciseGlobalPlanner()  {}

ExerciseGlobalPlanner::~ExerciseGlobalPlanner() {}

void ExerciseGlobalPlanner::get_input(nav_msgs::OccupancyGrid *map) // [change to] void ExerciseGlobalPlanner::initialize(nav_msgs::OccupancyGrid *map)
{
  map_ = map;
  ROS_INFO("EXERCISE GLOBAL PLANNER INITIALIZED");
}


asv_msgs::Path ExerciseGlobalPlanner::send_output(const double start_x, // [change to] void ExerciseGlobalPlanner::calculate_waypoints(const double start_x,
                                                  const double start_y,
                                                  const double arrival_x,
                                                  const double arrival_y)

  asv_msgs::Path wp;
  asv_msgs::Waypoint2D start;
  asv_msgs::Waypoint2D arrival;

  start.x = start_x;
  start.y = start_y;
  arrival.x = arrival_x;
  arrival.y = arrival_y;

  wp.waypoints.push_back(start);
  wp.waypoints.push_back(arrival);


  return wp;
}
