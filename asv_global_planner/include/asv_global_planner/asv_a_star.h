#pragma once

#ifndef ASV_A_STAR
#define ASV_A_STAR

#include "nav_msgs/OccupancyGrid.h"

#include "asv_global_planner/asv_global_planner.h"
#include "visualization_msgs/MarkerArray.h"

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
    visualization_msgs::MarkerArray calculate_waypoints(const double start_x, const double start_y, const double arrival_x, const double arrival_y);

    bool isValid(int x, int y);
    bool isObstacle(int x, int y);
    bool isDestination(int x, int y, Node dest);
    double calculateH(int x, int y, Node dest);
    visualization_msgs::MarkerArray aStar(Node player, Node dest);
    visualization_msgs::MarkerArray makePath(Node dest);

  private :
    int X_STEP_;
    int Y_STEP_;
    int X_MAX_;
    int Y_MAX_;
    std::vector<Node> allMap_;
};

#endif
