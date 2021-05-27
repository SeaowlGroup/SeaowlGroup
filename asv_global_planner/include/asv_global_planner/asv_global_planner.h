#pragma once

#ifndef ASV_GOBAL_PLANNER
#define ASV_GLOBAL_PLANNER

#include "nav_msgs/OccupancyGrid.h"
#include "asv_msgs/Path.h"

#include <vector>
#include <utility>

static const int OCCUPIED_TRESH = 40;

class GlobalPlanner
{
 public:
  GlobalPlanner();
  ~GlobalPlanner();
  virtual void initialize(nav_msgs::OccupancyGrid *map);
  virtual void reinit() = 0;
  virtual asv_msgs::Path calculate_waypoints(const double x, const double y, const double arrival_x, const double arrival_y) = 0;

 protected:

  // ROS API
  nav_msgs::OccupancyGrid *map_;
};


#endif
