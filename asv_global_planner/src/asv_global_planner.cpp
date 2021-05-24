#include "ros/ros.h"
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"

#include <iostream>


#include "asv_global_planner/asv_global_planner.h"

GlobalPlanner::GlobalPlanner()
{
}

GlobalPlanner::~GlobalPlanner()
{
}


void GlobalPlanner::initialize(nav_msgs::OccupancyGrid *map)
{
  map_ = map;
}
