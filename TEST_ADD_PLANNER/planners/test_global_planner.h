#pragma once

#ifndef TEST_GLOBAL_PLANNER
#define TEST_GLOBAL_PLANNER

#include "nav_msgs/OccupancyGrid.h"

#include "asv_msgs/Path.h"
#include "asv_msgs/Waypoint2D.h"

#include <vector>
#include <utility>

struct Node2
{
    int y;
    int x;
    int parentX;
    int parentY;
    float gCost;
    float hCost;
    float fCost;
};


class ExerciseGlobalPlanner :
{
  public :
    ExerciseGlobalPlanner();
    ~ExerciseGlobalPlanner();
    void init(nav_msgs::OccupancyGrid *map);
    void reinit();
    asv_msgs::Path get_path(const double start_x, const double start_y, const double arrival_x, const double arrival_y);

    bool isValid(int x, int y);
    bool isObstacle(int x, int y);
    bool isDestination(int x, int y, Node2 dest);
    double calculateH(int x, int y, Node2 dest);
    asv_msgs::Path aStar(Node2 player, Node2 dest);
    asv_msgs::Path makePath(Node2 dest);
    bool thereIsAWay(Node2 n_inf, Node2 n_sup);

  private :
    int X_STEP_;
    int Y_STEP_;
    int X_MAX_;
    int Y_MAX_;
    std::vector<Node2> allMap_;
    std::vector<bool> obstMap_;
    nav_msgs::OccupancyGrid *map_;
};

#endif
