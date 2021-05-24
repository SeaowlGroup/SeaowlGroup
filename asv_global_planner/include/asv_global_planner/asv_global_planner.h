#pragma once

#ifndef ASV_GOBAL_PLANNER
#define ASV_GLOBAL_PLANNER

#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"

#include <vector>
#include <utility>

static const int OCCUPIED_TRESH = 40;

class GlobalPlanner
{
 public:
  /// Constructor
  GlobalPlanner();
  /// Destructor
  ~GlobalPlanner();
  /**
   * @brief Initializes the controller.
   *
   * @param map A pointer to the occupancy grid published by the map_server.
   */
  virtual void initialize(nav_msgs::OccupancyGrid *map);
  /**
   * @brief Called after the velocity field has been updated to get the (u, psi) pair
   * with the lowest cost.
   *
   * @param u_best The reference parameter to store the "best" surge speed.
   * @param psi_best The reference parameter to store the "best" heading.
   */
  virtual visualization_msgs::MarkerArray calculate_waypoints(const double x, const double y, const double arrival_x, const double arrival_y) = 0;
  
 protected:  

  // ROS API
  nav_msgs::OccupancyGrid *map_;
};


#endif
