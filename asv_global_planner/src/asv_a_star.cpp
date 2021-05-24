#include "ros/ros.h"
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"

#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <utility>
#include <stack>

#include "asv_global_planner/asv_a_star.h"


AStarPlanner::AStarPlanner(int X_STEP, int Y_STEP) :  X_STEP_(X_STEP), Y_STEP_(Y_STEP) {}

AStarPlanner::~AStarPlanner() {}

void AStarPlanner::initialize(nav_msgs::OccupancyGrid *map) {
  map_ = map;
  X_MAX_ = map->info.width;
  Y_MAX_ = map->info.height;

  allMap_.resize(X_MAX_*Y_MAX_/(X_STEP_*Y_STEP_));

  for (int x = 0; x < (X_MAX_ / X_STEP_); x++) {
    for (int y = 0; y < (Y_MAX_ / Y_STEP_); y++) {
      int id = x + y * (X_MAX_ / X_STEP_);
      allMap_[id].fCost = FLT_MAX; //FLT_MAX is the biggest computable float, it represents the infinite cost
      allMap_[id].gCost = FLT_MAX;
      allMap_[id].hCost = FLT_MAX;
      allMap_[id].parentX = -1;
      allMap_[id].parentY = -1;
      allMap_[id].x = x;
      allMap_[id].y = y;
    }
  }
}

bool AStarPlanner::isValid(int x, int y) { //If our Node is an obstacle it is not valid
  if (x < 0 || y < 0 || x >= (X_MAX_ / X_STEP_) || y >= (Y_MAX_ / Y_STEP_)) {
    return false;
  }
  else {
    if (!isObstacle(x, y)) {
      return true;
    }
    else {
      return false;
    } 
  }
}

bool AStarPlanner::isObstacle(int x, int y)
{
  int count = 0;
  for(int i=0; i<=X_STEP_; i=i+1) {
    for(int j=0; j<=Y_STEP_; j=j+1) {
      //int pixel = map_->data[i + j*map_->info.width];
      int pixel = map_->data[x*X_STEP_ + i + (y*Y_STEP_ + j)*map_->info.width];
      if (pixel > OCCUPIED_TRESH) return true;
    }
  }
  return false;
}


bool AStarPlanner::isDestination(int x, int y, Node dest) {
  if (x == dest.x && y == dest.y) {
    return true;
  }
  return false;
}

double AStarPlanner::calculateH(int x, int y, Node dest) {
  double H = (sqrt((x - dest.x)*(x - dest.x) + (y - dest.y)*(y - dest.y)));
  return H;
}

visualization_msgs::MarkerArray AStarPlanner::aStar(Node player, Node dest) {
  visualization_msgs::MarkerArray empty;
  if (isValid(dest.x, dest.y) == false) {
      ROS_ERROR("GLOBAL PLANNER FAIL : Destination is invalid (x = %d, y = %d)", dest.x, dest.y);
      return empty;
      //Destination is invalid
  }
  if (isDestination(player.x, player.y, dest)) {
    return empty;
    //You clicked on yourself
  }
  bool closedList[(X_MAX_ / X_STEP_)][(Y_MAX_ / Y_STEP_)];
  //Initialize whole map
  //Node allMap_[50][25];
  for (int x = 0; x < (X_MAX_ / X_STEP_); x++) {
    for (int y = 0; y < (Y_MAX_ / Y_STEP_); y++) {
      closedList[x][y] = false;
    }
  }

  //Initialize our starting list
  int x = player.x;
  int y = player.y;
  allMap_[x + y * (X_MAX_ / X_STEP_)].fCost = 0.0;
  allMap_[x + y * (X_MAX_ / X_STEP_)].gCost = 0.0;
  allMap_[x + y * (X_MAX_ / X_STEP_)].hCost = 0.0;
  allMap_[x + y * (X_MAX_ / X_STEP_)].parentX = x;
  allMap_[x + y * (X_MAX_ / X_STEP_)].parentY = y;

  std::vector<Node> openList;  
  openList.emplace_back(allMap_[x + y * (X_MAX_ / X_STEP_)]);
  bool destinationFound = false;
  while (!openList.empty()&&openList.size()<(X_MAX_ / X_STEP_)*(Y_MAX_ / Y_STEP_)) {
      Node node;
    do {
      //This do-while loop could be replaced with extracting the first
      //element from a set, but you'd have to make the openList a set.
      //To be completely honest, I don't remember the reason why I do
      //it with a vector, but for now it's still an option, although
      //not as good as a set performance wise.
      float temp = FLT_MAX;
      std::vector<Node>::iterator itNode;
      for (std::vector<Node>::iterator it = openList.begin(); it != openList.end(); it = next(it)) {
        Node n = *it;
        if (n.fCost < temp) {
          temp = n.fCost;
          itNode = it;
        }
      }
      node = *itNode;
      openList.erase(itNode);
    } while (isValid(node.x, node.y) == false);

    x = node.x;
    y = node.y;
    closedList[x][y] = true;

    //For each neighbour starting from North-West to South-East
    for (int newX = -1; newX <= 1; newX++) {
      for (int newY = -1; newY <= 1; newY++) {
        double gNew, hNew, fNew;
	int id = x + newX + (y + newY)*(X_MAX_ / X_STEP_);
        if (isValid(x + newX, y + newY)) {
          if (isDestination(x + newX, y + newY, dest)) {
            //Destination found - make path 
            allMap_[id].parentX = x;
            allMap_[id].parentY = y;
            destinationFound = true;
            return makePath(dest);
          }
          else if (closedList[x + newX][y + newY] == false) {
            if(newX == newY){
	      gNew = node.gCost + 1.414;
	    } else {
	      gNew = node.gCost + 1.0;
	    }
            hNew = calculateH(x + newX, y + newY, dest);
            fNew = gNew + hNew;
            // Check if this path is better than the one already present
            if (allMap_[id].fCost == FLT_MAX || allMap_[id].fCost > fNew) {
              // Update the details of this neighbour node
              allMap_[id].fCost = fNew;
              allMap_[id].gCost = gNew;
              allMap_[id].hCost = hNew;
              allMap_[id].parentX = x;
              allMap_[id].parentY = y;
              openList.emplace_back(allMap_[id]);
            }
          }
        }
      }
    }
  }
  if (destinationFound == false) {
    ROS_ERROR("GLOBAL PLANNER FAIL : Destination not found");
    return empty;
  }
}

visualization_msgs::MarkerArray AStarPlanner::makePath(Node dest) {
  //try {
    ROS_INFO("GLOBAL PLANNER SUCCESS : Found a path");
    int x = dest.x;
    int y = dest.y;
    int id = x + y*(X_MAX_ / X_STEP_);
    std::stack<Node> path;
    visualization_msgs::MarkerArray usablePath;

    while (!(allMap_[id].parentX == x && allMap_[id].parentY == y) && allMap_[id].x != -1 && allMap_[id].y != -1) 
    {
      path.push(allMap_[id]);
      int tempX = allMap_[id].parentX;
      int tempY = allMap_[id].parentY;
      x = tempX;
      y = tempY;
      id = x + y*(X_MAX_ / X_STEP_);
    }
    path.push(allMap_[id]);

    while (!path.empty()) {
      Node top = path.top();
      visualization_msgs::Marker coord;
      coord.pose.position.x = X_STEP_*(top.x + 0.5);
      coord.pose.position.y = Y_STEP_*(top.y + 0.5);
      path.pop();    
     
      usablePath.markers.emplace_back(coord);
    }
    return usablePath;
  //}
  //catch(const exception& e){
  //  ROS_ERROR(e.what());
  //}
}


visualization_msgs::MarkerArray AStarPlanner::calculate_waypoints(const double start_x, const double start_y, const double arrival_x, const double arrival_y)
{
  // map_->data[px_i + py_i*map_->info.width]

  Node player;
  Node dest;
  visualization_msgs::MarkerArray wp;

  player.x = (int) start_x / X_STEP_;
  player.y = (int) start_y / Y_STEP_;
  dest.x = (int) arrival_x / X_STEP_;
  dest.y = (int) arrival_y / Y_STEP_;

  //wp = [[arrival_x, arrival_y]];
  //wp.emplace_back(aStar(player, dest));
  
  wp = aStar(player, dest);

  //std::vector<double> test = {0.5, 0.9};
  
  //ros::param::set("/asv/LOSNode/waypoints", test);

  return wp;
}

///////////////UTILS//////////////////////

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}
