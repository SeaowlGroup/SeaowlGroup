#include "ros/ros.h"
#include <ros/console.h>

#include <ctime>
#include <cmath>

#include "asv_map_processing/asv_map_processing_node.h"


const int OCCUPANCY_THRESHOLD = 40; //à paramétrer


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "asv_map_processing_node");
  ros::start();

  ROS_INFO("Starting map processing node!");

  ros::NodeHandle n;

  MapProcessingNode mp_node;


  ros::Publisher inflated_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/processed_map", 1, true);


  ros::Subscriber map_sub = n.subscribe("/map",
                                       1,
                                       &MapProcessingNode::mapCallback,
                                       &mp_node);


  mp_node.initialize(&inflated_map_pub, &map_sub);

  ros::Rate loop_rate(10.0);
  clock_t tick, tock;

  while (ros::ok())
    {
      // For timing of algorithm: uncomment!
      //tick = clock();

      mp_node.publish();

      //tock = clock();
      //ROS_INFO("Loop speed: %.2f ms", ((float) (tock-tick)/CLOCKS_PER_SEC * 1e3 ));
      ros::spinOnce();
      loop_rate.sleep();
    }
  ros::shutdown();
  return 0;
}

MapProcessingNode::MapProcessingNode() : inflated_map_pub_(NULL), map_sub_(NULL) {}

MapProcessingNode::~MapProcessingNode() {}

void MapProcessingNode::publish()
{
  if (map_.data.size() > 0) {
    if (!processed_ && map_.data.size() > 0) {
      inflate();
      processed_ = true;
    }
    if (!sent) {
     inflated_map_pub_->publish(map_);
     sent = true;
   }
  }
}

void MapProcessingNode::initialize(ros::Publisher *inflated_map_pub, ros::Subscriber *map_sub)
{
  inflated_map_pub_ = inflated_map_pub;
  map_sub_ = map_sub;
  ros::param::get("~coast_margin", margin_);
  margin_++;
}

void MapProcessingNode::inflate() {
  for(int x=0; x < map_.info.width; x++) {
    for(int y=0; y < map_.info.height; y++) {
      if(hasObstacleNear(x, y)) {
	      map_.data[x + y*map_.info.width] = 100;
      }
    }
  }
}

bool MapProcessingNode::hasObstacleNear(const int x, const int y)
{
  int range = ceil(margin_/map_.info.resolution);

  for(int i=x-range; i <= x+range; i++) {
    for(int j=y-range; j <= y+range; j++) {
      if (i < map_.info.width && j < map_.info.height && i >= 0 && j >= 0) {
        int id = i + j*map_.info.width;
        if (originalMap_[id] >= OCCUPANCY_THRESHOLD && isNear(x, y, i, j)) return true;
        //if (originalMap_[id] >= OCCUPANCY_THRESHOLD) return true;
      }
    }
  }
  return false;
}

bool MapProcessingNode::isNear(const int xi, const int yi, const int xj, const int yj)
{
  double dist = sqrt((xi-xj)*(xi-xj) + (yi-yj)*(yi-yj))*map_.info.resolution;
  return (dist < margin_);
}

void MapProcessingNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  // Copy what we need
  map_.info.resolution = msg->info.resolution;
  map_.info.height = msg->info.height;
  map_.info.width = msg->info.width;
  map_.info.origin.position.x = msg->info.origin.position.x;
  map_.info.origin.position.y = msg->info.origin.position.y;
  map_.data.resize(msg->data.size());
  std::vector<signed char, std::allocator<signed char>>::iterator it;
  for(it = map_.data.begin(); it < map_.data.end(); it++) {
    *it = 0;
  }
  originalMap_ = msg->data;
}
