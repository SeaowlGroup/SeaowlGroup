#include "ros/ros.h"
#include <ros/console.h>
#include <cmath>

#include "asv_global_planner/asv_global_planner_node.h"


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "asv_global_planner_node");
  ros::start();

  ROS_INFO("Starting global planner node!");

  ros::NodeHandle n;

  GlobalPlannerNode gp_node;
  //GlobalPlanner *gp = new GlobalPlanner;
  AStarPlanner *gp = new AStarPlanner(10, 10);


  ros::Publisher wp_pub = n.advertise<visualization_msgs::MarkerArray>("asv_waypoints", 1);


  ros::Subscriber og_sub = n.subscribe("/processed_map",
                                       1,
                                       &GlobalPlannerNode::mapCallback,
                                       &gp_node);

  ros::Subscriber asv_sub = n.subscribe("state",
                                        1,
                                        &GlobalPlannerNode::asvCallback,
                                        &gp_node);

  gp_node.initialize(&wp_pub, &og_sub, &asv_sub, gp);
  gp_node.start();

  ros::shutdown();
  return 0;
}


GlobalPlannerNode::GlobalPlannerNode() : gp_(NULL),
                                         wp_pub_(NULL),
                                         og_sub_(NULL),
                                         asv_sub_(NULL) {};

GlobalPlannerNode::~GlobalPlannerNode() {}

void GlobalPlannerNode::initialize(ros::Publisher *wp_pub,
                                      ros::Subscriber *og_sub,
                                      ros::Subscriber *asv_sub,
                                      GlobalPlanner *gp)
{
  wp_pub_ = wp_pub;
  og_sub_ = og_sub;
  asv_sub_ = asv_sub;

  gp_ = gp;
}

void GlobalPlannerNode::start()
{
  ros::Rate loop_rate(10.0);
  clock_t tick, tock;

  visualization_msgs::MarkerArray waypt;

  while (ros::ok())
    {
      if (map_init == 1) {
          gp_->initialize(&map_);
          ROS_INFO("Global planner initialized");
	        waypt = gp_->calculate_waypoints(start_x, start_y, 770.0, 580.0);
	        map_init = 2;
      }

      // For timing of algorithm: uncomment!
      //tick = clock();

      //gp_->update();

      //waypt.markers[n].pose.position;
      //visualization_msgs::MarkerArray waypt = gp_->calculate_waypoints(start_x, start_y, 0.0, 0.0);

      //marker_.header.stamp = ros::Time();
      if (waypt.markers.size() > 0) {
        wp_pub_->publish(waypt);
      }

      //tock = clock();
      //ROS_INFO("Loop speed: %.2f ms", ((float) (tock-tick)/CLOCKS_PER_SEC * 1e3 ));
      ros::spinOnce();
      loop_rate.sleep();
    }
}


void GlobalPlannerNode::asvCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  //gp_->updateAsvState(msg, u_d_, psi_d_);
  start_x = msg->pose.pose.position.x;
  start_y = msg->pose.pose.position.y;
}



void GlobalPlannerNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  // Copy what we need
  map_.info.resolution = msg->info.resolution;
  map_.info.height = msg->info.height;
  map_.info.width = msg->info.width;
  map_.info.origin.position.x = msg->info.origin.position.x;
  map_.info.origin.position.y = msg->info.origin.position.y;

  ROS_INFO("r %f, h %d, w%d, px %f, py %f",
           map_.info.resolution,
           map_.info.height,
           map_.info.width,
           map_.info.origin.position.x,
           map_.info.origin.position.y);

  map_.data = msg->data;
  map_init = 1;
}
