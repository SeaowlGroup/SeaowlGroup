#pragma once

#ifndef TEST_GLOBAL_PLANNER
#define TEST_GLOBAL_PLANNER

#include "nav_msgs/OccupancyGrid.h"

#include "asv_msgs/Path.h"
#include "asv_msgs/Waypoint2D.h"

// [add] #include "asv_global_planner/asv_global_planner.h" 


class ExerciseGlobalPlanner : // [change to] class ExerciseGlobalPlanner : public GlobalPlanner
{
  public :
    ExerciseGlobalPlanner();
    ~ExerciseGlobalPlanner();

    // The two main methods are used to respectively initialize the planner and compute the path
    void get_input(nav_msgs::OccupancyGrid *map); // [change to] void initialize(nav_msgs::OccupancyGrid *map);
    asv_msgs::Path send_output(const double start_x, // [change to] asv_msgs::Path calculate_waypoints(const double start_x,
                               const double start_y,
                               const double arrival_x,
                               const double arrival_y);

  private :
    nav_msgs::OccupancyGrid *map_;
};

#endif
