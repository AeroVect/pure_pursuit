#include "pure_pursuit/pure_pursuit.h"

vec_control::PurePursuit::PurePursuit() 
  : last_pose_stream(last_pose_csv)
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");
  // Node paramters
  nh_private.param<double>("ld_gain", ld_gain_, 1.0);
  nh_private.param<double>("min_ld", min_ld_, 0.5);
  nh_private.param<double>("car_wheel_base", car_wheel_base_, 0.44);
  nh_private.param<int>("controller_freq", controller_freq_, 10);
  nh_private.param<int>("n_laps", n_laps_, 0);
  nh_private.param<double>("distance_thresh", distance_thresh_, 1.0);
  nh_private.param<double>("turning_angle", turning_angle_, 30.0);
  nh_private.param<bool>("use_closest_point", use_closest_point_, false);
  nh_private.param<std::string>("map_frame", map_frame_, "earth");
  nh_private.param<std::string>("base_frame", base_frame_, "base_link");
  nh_.param<std::string>("/wps_player/last_pose_csv", last_pose_csv, "/ros_ws/latest_pose.csv");    
  ld_ = min_ld_;
  ros_rate_ = new ros::Rate(controller_freq_);
  // Publishers and subscribers
  control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/pure_pursuit/control", 1);
  ros::Subscriber odom_sub_ =
      nh_.subscribe("/odom", 1, &PurePursuit::odom_clk_, this);
  ros::Subscriber path_sub_ =
      nh_.subscribe("/pure_pursuit/path", 1, &PurePursuit::path_clk_, this);
  ros::Subscriber obstacles_flag_ =
      nh_.subscribe("/speed_flag", 1, &PurePursuit::obstacles_flag_clk_, this);
  end_state_pub_ = nh_.advertise<std_msgs::Empty>("/pure_pursuite/stop_signal", 1);
  tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
  l_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/pure_pursuit/lookahead_point", 1);
  left_turn_pub_ = nh_.advertise<std_msgs::Bool>("/left_turn_flash", 1);
  right_turn_pub_ = nh_.advertise<std_msgs::Bool>("/right_turn_flash", 1);
  // main loop
  control_loop_();
}
void vec_control::PurePursuit::odom_clk_(
    const nav_msgs::Odometry::ConstPtr &msg) {
  car_speed_ = msg->twist.twist.linear.x;
  ld_ = std::max(ld_gain_ * car_speed_, min_ld_);
  last_x_pose = msg -> pose.pose.position.x;
  last_y_pose = msg -> pose.pose.position.y;  
  last_pose_stream << last_x_pose <<","<< last_y_pose<<"\n";
}
void vec_control::PurePursuit::path_clk_(const nav_msgs::Path::ConstPtr &msg) {
  ROS_INFO("New path is received.");
  path_ = msg->poses;
  // path_.push_back(msg->poses[0]);
  got_path_ = true;
  path_done_ = false;
  point_idx_ = 0;
  closest_point_idx_ = 5;
  double start_end_dist =
      distance(path_[0].pose.position, path_.back().pose.position);
  ROS_INFO("Start to End Distance: %f", start_end_dist);
  ROS_INFO("Min lookup distance: %f", min_ld_);
  if (start_end_dist > min_ld_ && n_laps_ > 0) {
    ROS_WARN("Ignoring N laps. start and end points are too far !");
    n_laps_ = 0;
  }
}

void vec_control::PurePursuit::obstacles_flag_clk_(
    const aerovect_msgs::SpeedFlags::ConstPtr &msg) {
  stop_flag_ = msg->flag;

  ROS_INFO("Got Stop Flag %d", stop_flag_);
}

void vec_control::PurePursuit::control_loop_() {
  double y_t = 0, ld_2 = 0, delta = 0;
  double distance_ = 0;
  double target_speed = 0;
  while (ros::ok()) {
    if (got_path_) {
      // get the current robot location by tf base_link -> map
      // iterate over the path points
      // if the distance between a point and robot > lookahead break and take
      // this point transform this point to the robot base_link the y component
      // of this point is y_t delta can be computed as atan2(2 * yt * L_, ld_2)
      try {
        base_location_ = tfBuffer_.lookupTransform(
            map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));

        for (; point_idx_ < path_.size(); point_idx_++) {
          distance_ = distance2d(path_[point_idx_].pose.position,
                                 base_location_.transform.translation);
          ROS_INFO("Lookahead Point ID: %d, Distance %f", point_idx_, distance_);
          if (distance_ >= ld_) {
            path_[point_idx_].header.stamp =
                ros::Time::now(); // Set the timestamp to now for the transform
                                  // to work, because it tries to transform the
                                  // point at the time stamp of the input point
            float tmp = path_[point_idx_].pose.position.z;
            path_[point_idx_].pose.position.z = 0;
            tfBuffer_.transform(path_[point_idx_], target_point_, base_frame_,
                                ros::Duration(0.1));
            path_[point_idx_].pose.position.z = tmp;
            break;
          }
        }
        if(use_closest_point_){
        // Find the closest point to the vehicle right now 
        // and take it's speed as target speed
        for(;closest_point_idx_ < path_.size(); closest_point_idx_++){
          double distance = distance2d(path_[closest_point_idx_].pose.position,
                                 base_location_.transform.translation);
          if (distance <= distance_thresh_){
            target_speed = path_[closest_point_idx_].pose.position.z;
            ROS_INFO("Closest Point ID: %d, Distance %f", closest_point_idx_, distance);
            break;
          }
        }
        if(closest_point_idx_ >= point_idx_){
          ROS_WARN("Lookahead Point is behind the vehicle !!");
          ROS_WARN("Lookahead Point ID:%d Closest point ID: %d", point_idx_, closest_point_idx_);
        }
        }else{
          if(point_idx_ == path_.size()){
            target_speed = 0.5;
          }else{
            target_speed = path_[point_idx_].pose.position.z;
          }
          
        }
        // check stop flag
        if (stop_flag_ == 2) {
          target_speed = 0;
          ROS_INFO("Stopping due to speed flag 2");
        } else if (stop_flag_ == 1) {
          target_speed /= 2;
          ROS_INFO("slowing due to speed flag 1");
          ROS_INFO("Current Speed %f", target_speed);
        }
        // Calculate the steering angle
        ld_2 = ld_ * ld_;
        y_t = target_point_.pose.position.y;
        delta = atan2(2 * car_wheel_base_ * y_t, ld_2);
        control_msg_.drive.steering_angle = delta;
        control_msg_.drive.speed = target_speed;
        control_msg_.header.stamp = ros::Time::now();
        control_pub_.publish(control_msg_);
        // Check turning signals
        double angle = 180 * atan2(target_point_.pose.position.y,target_point_.pose.position.x) / M_PI;
        ROS_INFO("Lookahead Angle: %f", angle);
        std_msgs::Bool turning_msg;
        if(abs(angle)>= turning_angle_){
          turning_msg.data = true;
          if (angle > 0 ){
            ROS_INFO("Turning left");
            left_turn_pub_.publish(turning_msg);
            turning_msg.data = false;
            right_turn_pub_.publish(turning_msg);
          }else{
            ROS_INFO("Turning right");
            right_turn_pub_.publish(turning_msg);
            turning_msg.data = false;
            left_turn_pub_.publish(turning_msg);
          }
        }else{
          turning_msg.data = false;
          left_turn_pub_.publish(turning_msg);
          right_turn_pub_.publish(turning_msg);
        }

        last_p_idx_ = point_idx_;
        last_dist_ = distance_;
        if(point_idx_ == path_.size()) {
          n_laps_--;
          if (n_laps_ > 0) {
            point_idx_ = 0;
            closest_point_idx_ = 0;
          } else {
            distance_ = distance2d(path_[path_.size()-1].pose.position,
                                 base_location_.transform.translation); 
            ROS_INFO("Reached final point. Distance to the final point: %f", distance_);
            if(distance_<= distance_thresh_){
            ROS_INFO("Final point distance: %f", distance_);
            control_msg_.drive.steering_angle = 0;
            control_msg_.drive.speed = 0;
            control_msg_.header.stamp = ros::Time::now();
            control_pub_.publish(control_msg_);
            got_path_ = false;
            point_idx_ = 0;
            closest_point_idx_ = 5;
            std_msgs::Empty empty_msg;
            end_state_pub_.publish(empty_msg);    
            }            
                    
          }
        }
        try {
          lookahead_p.point = path_[point_idx_].pose.position;
          lookahead_p.header = path_[point_idx_].header;
          l_point_pub_.publish(lookahead_p); // Publish the lookahead point
        } catch (...) {

        }
        
      } 
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }
    }

    ros::spinOnce();
    ros_rate_->sleep();
  }
}

vec_control::PurePursuit::~PurePursuit() {
  delete tfListener_;
  last_pose_stream.close();
  delete ros_rate_;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit");
  vec_control::PurePursuit pp_node;
}