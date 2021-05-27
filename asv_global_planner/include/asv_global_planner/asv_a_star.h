#pragma once

#ifndef ASV_A_STAR
#define ASV_A_STAR

#include "nav_msgs/OccupancyGrid.h"

#include "asv_global_planner/asv_global_planner.h"
#include "asv_msgs/Path.h"
#include "asv_msgs/Waypoint2D.h"

#include <vector>
#include <utility>

struct Node
{
    int y;
    int x;
    int parentX;
    int parentY;
    float gCost;
    float hCost;
    float fCost;
};


class AStarPlanner : public GlobalPlanner
{
  public :
    AStarPlanner(int X_STEP = 10, int Y_STEP = 10);
    ~AStarPlanner();
    void initialize(nav_msgs::OccupancyGrid *map);
    void reinit();
    asv_msgs::Path calculate_waypoints(const double start_x, const double start_y, const double arrival_x, const double arrival_y);

    bool isValid(int x, int y);
    bool isObstacle(int x, int y);
    bool isDestination(int x, int y, Node dest);
    double calculateH(int x, int y, Node dest);
    asv_msgs::Path aStar(Node player, Node dest);
    asv_msgs::Path makePath(Node dest);
    bool thereIsAWay(Node n_inf, Node n_sup);

  private :
    int X_STEP_;
    int Y_STEP_;
    int X_MAX_;
    int Y_MAX_;
    std::vector<Node> allMap_;
    std::vector<bool> obstMap_;
};

#endif
