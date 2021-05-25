#ifndef ASV_MAP_PROCESSING_NODE
#define ASV_MAP_PROCESSING_NODE

#include <vector>

#include "nav_msgs/OccupancyGrid.h"


class MapProcessingNode
{
 public:
  /// Constructor
  MapProcessingNode();
  /// Destructor
  ~MapProcessingNode();

  void initialize(ros::Publisher *inflated_map_pub,
                  ros::Subscriber *map_sub);

  void inflate();
  bool isNear(const int xi, const int yi, const int xj, const int yj);
  bool hasObstacleNear(const int x, const int y);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void publish();

 private:
  const double margin_ = 10.0; //à paramétrer
  nav_msgs::OccupancyGrid map_;
  bool processed_ = false;
  bool sent = false;
  std::vector<signed char, std::allocator<signed char>> originalMap_ ;

  // ROS API
  ros::Publisher *inflated_map_pub_;
  ros::Subscriber *map_sub_;
};


#endif
