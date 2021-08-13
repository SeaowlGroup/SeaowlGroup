#ifndef TEST_LOCAL_PLANNER
#define TEST_LOCAL_PLANNER

#include <eigen3/Eigen/Dense>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"


class ExerciseLocalPlanner
{
 public:
  ExerciseLocalPlanner();
  ~ExerciseLocalPlanner();

  void init(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map, ros::NodeHandle nh);
  // [change to] void initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map, ros::NodeHandle nh);
  void update_AsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d);
  // [change to] void updateAsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d);
  void get_BestControlInput(double &u_best, double &psi_best);
  // [change to] void getBestControlInput(double &u_best, double &psi_best);

 private:
  double u_d_;
  double psi_d_;

  std::vector<asv_msgs::State> *obstacles_;
  nav_msgs::OccupancyGrid *map_;
};


#endif
