#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"

#include <iostream>
#include <vector>

#include "asv_ctrl_vo/test_local_planner.h"

ExerciseLocalPlanner::ExerciseLocalPlanner() {}
ExerciseLocalPlanner::~ExerciseLocalPlanner() {}

// [add] void ExerciseLocalPlanner::update() {}
// [add] void ExerciseLocalPlanner::initializeMarker(visualization_msgs::Marker *marker) {}

void ExerciseLocalPlanner::init(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map, ros::NodeHandle nh)
// [change to] void ExerciseLocalPlanner::initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map, ros::NodeHandle nh)
{
  obstacles_ = obstacles;
  map_ = map;
  ROS_INFO("EXERCISE LOCAL PLANNER INITIALIZED");
}

void ExerciseLocalPlanner::update_AsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d)
// [change to] void ExerciseLocalPlanner::update_AsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d)
{
  u_d_ = u_d;
  psi_d_ = psi_d;
}

void ExerciseLocalPlanner::get_BestControlInput(double &u_best, double &psi_best)
// [change to] void ExerciseLocalPlanner::getBestControlInput(double &u_best, double &psi_best)
{
  u_best = u_d_;
  psi_best = psi_d_;
}
