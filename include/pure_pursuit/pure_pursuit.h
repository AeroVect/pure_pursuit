#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "aerovect_msgs/SpeedFlags.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <algorithm>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <limits>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>

namespace vec_control {
class PurePursuit {
private:
  double ld_gain_;
  double ld_;
  double min_ld_;
  double car_wheel_base_;
  double alpha_;
  double car_speed_;
  int controller_freq_;
  int point_idx_ = 0;
  int closest_point_idx_ = 0;
  double distance_thresh_ = 0.1;
  int last_p_idx_;
  int n_laps_;
  double last_dist_ = std::numeric_limits<double>::infinity();
  bool got_path_ = false;
  bool path_done_ = true;
  int stop_flag_ = 0;
  double turning_angle_;
  bool use_closest_point_;
  std::string map_frame_ = "earth";
  std::string base_frame_ = "base_link";
  ros::Time last_msg_time_;
  std::vector<geometry_msgs::PoseStamped> path_;
  geometry_msgs::PoseStamped target_point_;
  ackermann_msgs::AckermannDriveStamped control_msg_;
  geometry_msgs::PointStamped lookahead_p;
  geometry_msgs::PoseArray lidar_obstacles_;
  bool new_obstacle_received_ = false;
  int obj_lookahead_;
  double obj_waypt_distance_threshold_m_;

  ros::Publisher control_pub_;
  ros::Publisher l_point_pub_;
  ros::Publisher current_speed_pub_;
  ros::Publisher end_state_pub_;
  ros::Publisher left_turn_pub_;
  ros::Publisher right_turn_pub_;
  ros::Subscriber ackermann_sub_;
  geometry_msgs::TransformStamped base_location_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener *tfListener_;

  ros::Rate *ros_rate_;

  void odom_clk_(const nav_msgs::Odometry::ConstPtr &msg);
  void path_clk_(const nav_msgs::Path::ConstPtr &msg);
  void obstacles_flag_clk_(const aerovect_msgs::SpeedFlags::ConstPtr &msg);
  void lidar_obstacles_cb(const geometry_msgs::PoseArray::ConstPtr &msg);
  void control_loop_();

public:
  PurePursuit();
  ~PurePursuit();
};

template <typename T1, typename T2> double distance(T1 pt1, T2 pt2) {
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
              pow(pt1.z - pt2.z, 2));
}

template <typename T1, typename T2> double distance2d(T1 pt1, T2 pt2) {
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

} // namespace vec_control

#endif