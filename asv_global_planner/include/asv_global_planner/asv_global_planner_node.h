#pragma once

#ifndef ASV_GLOBAL_PLANNER_NODE
#define ASV_GLOBAL_PLANNER_NODE

#include "asv_msgs/StateArray.h"
#include "nav_msgs/Odometry.h"
#include "asv_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>

#include "asv_global_planner/asv_global_planner.h"
#include "asv_global_planner/asv_a_star.h"

class GlobalPlannerNode
{
 public:
  /// Constructor
  GlobalPlannerNode();
  /// Destructor
  ~GlobalPlannerNode();

  /**
   * Initializes the Global Planner node
   *
   * @param cmd_pub
   * @param og_sub
   * @param asv_sub
   * @param gp
   */
  void initialize(ros::Publisher *wp_pub,
                  ros::Subscriber *og_sub,
                  ros::Subscriber *asv_sub,
                  ros::Subscriber *goal_sub,
                  GlobalPlanner *gp);
  /**
   * Start the node. Enters a "never ending" while loop.
   */
  void start();


  void asvCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
 private:
  double start_x;
  double start_y;
  GlobalPlanner *gp_;

  nav_msgs::OccupancyGrid map_;
  int map_init = 0; //0 means the map isn't initialized, 1 means the node has initialized its map but not the planner, 2 means the node and the planner had it initialized
  //double goal_x_;
  //double goal_y_;

  // ROS API
  ros::Publisher *wp_pub_;

  ros::Subscriber *og_sub_;
  ros::Subscriber *asv_sub_;
  ros::Subscriber *goal_sub_;
};


#endif
