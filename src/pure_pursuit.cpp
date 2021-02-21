#include "pure_pursuit/pure_pursuit.h"

vec_control::PurePursuit::PurePursuit() 
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");
  // Node paramters
  nh_private.param<double>("ld_gain", ld_gain_, 1.0);
  nh_private.param<double>("min_ld", min_ld_, 0.5);
  nh_private.param<double>("car_wheel_base", car_wheel_base_, 0.44);
  nh_private.param<int>("controller_freq", controller_freq_, 10);
  nh_private.param<int>("n_laps", n_laps_, 0);
  nh_private.param<std::string>("map_frame", map_frame_, "earth");
  nh_private.param<std::string>("base_frame", base_frame_, "base_link");
  nh_private.param<int>("obj_lookahead", obj_lookahead_, 20);
  nh_private.param<double>("obj_waypt_distance_threshold_m", obj_waypt_distance_threshold_m_, 3.0);

  ld_ = min_ld_;
  ros_rate_ = new ros::Rate(controller_freq_);
  // Publishers and subscribers
  control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/pure_pursuit/control", 1);
  ros::Subscriber odom_sub_ =
      nh_.subscribe("/odom", 1, &PurePursuit::odom_clk_, this);
  ros::Subscriber path_sub_ =
      nh_.subscribe("/pure_pursuit/path", 1, &PurePursuit::path_clk_, this);
  
  // *****************************************************************************//
  // Disabling the stop flag subscriber as we no longer use the cost map
  // ros::Subscriber obstacles_flag_ =
  //     nh_.subscribe("/speed_flag", 1, &PurePursuit::obstacles_flag_clk_, this);

  ros::Subscriber lidar_obstacles_ = 
      nh_.subscribe("/lidar_perception/detected_objects", 1, &PurePursuit::lidar_obstacles_cb, this);
  tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
  l_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/pure_pursuit/lookahead_point", 1);
  // main loop
  control_loop_();
}

void vec_control::PurePursuit::odom_clk_(
    const nav_msgs::Odometry::ConstPtr &msg) {
  car_speed_ = msg->twist.twist.linear.x;
  ld_ = std::max(ld_gain_ * car_speed_, min_ld_);  
}

void vec_control::PurePursuit::lidar_obstacles_cb(
  const geometry_msgs::PoseArray::ConstPtr &msg) {
  ROS_INFO_ONCE("[Pure pursuit] Subscribing to the obstacle msgs");
  if (nullptr != msg) {
    lidar_obstacles_ = *msg;
    new_obstacle_received_ = true;
  }
}

void vec_control::PurePursuit::path_clk_(const nav_msgs::Path::ConstPtr &msg) {
  ROS_INFO("New path is received.");
  path_ = msg->poses;
  // path_.push_back(msg->poses[0]);
  got_path_ = true;
  path_done_ = false;
  point_idx_ = 0;
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
        // check stop flag
        // Forcing all the stop flag checks to be false as we no longer use
        // the cost map based obstable detection
        if (stop_flag_ == 2 && false) {
          target_speed = 0;
          ROS_INFO("Stopping due to speed flag 2");
        } else if (stop_flag_ == 1 && false) {
          target_speed /= 2;
          ROS_INFO("slowing due to speed flag 1");
          ROS_INFO("Current Speed %f", target_speed);
        } else {
          bool obs_tf_lookup_success = false;
          bool object_on_path = false;
          tf2::Stamped<tf2::Transform> obs_tf;

          if (new_obstacle_received_) {
            const auto header = lidar_obstacles_.header;
            std::string err;
            if (tfBuffer_.canTransform(
                  map_frame_, header.frame_id, header.stamp, ros::Duration(0.1), &err)) {
              const auto velo_earth = tfBuffer_.lookupTransform(map_frame_, header.frame_id, header.stamp, ros::Duration(0.1));
              tf2::fromMsg(velo_earth, obs_tf);
              obs_tf_lookup_success = true;
            } else {
              ROS_WARN("Obstacles tf lookup failed: %s", err.c_str());
            }
            new_obstacle_received_ = false;
          }

          base_location_ = tfBuffer_.lookupTransform(
              map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
          for (; point_idx_ < path_.size();) {
            bool valid_tf = (!obs_tf.getOrigin().fuzzyZero()) && obs_tf_lookup_success;
            if (valid_tf) {
              for (const auto & obj_pose : lidar_obstacles_.poses) {
                const auto position = obj_pose.position;
                tf2::Vector3 obj_pose_earth (position.x, position.y, position.z);
                obj_pose_earth = obs_tf * obj_pose_earth;
                const auto obj_pose_earth_msg = tf2::toMsg(obj_pose_earth);
                const auto end_it = (point_idx_ + obj_lookahead_ < path_.size()) ?
                  obj_lookahead_ : (path_.size() - 1);
                for (auto it = point_idx_; it < end_it; ++it) {
                  const auto norm2_dist = distance2d(
                    path_[point_idx_].pose.position, obj_pose_earth_msg);
                  if (norm2_dist < obj_waypt_distance_threshold_m_) {
                    object_on_path = true;
                    break;
                  }
                }
                if (object_on_path) {
                  break;
                }
              }
            }

            if (object_on_path) {
              target_speed = 0.0;
              ROS_INFO("Stopping due to object on path at location <%f, %f>",
                path_[point_idx_].pose.position.x, path_[point_idx_].pose.position.y);
              delta = 0.0;
              break;
            } else {
              point_idx_++;
              distance_ = distance2d(path_[point_idx_].pose.position,
                                  base_location_.transform.translation);
              ROS_INFO("Point ID: %d, Distance %f", point_idx_, distance_);
              if (distance_ >= ld_) {
                path_[point_idx_].header.stamp =
                    ros::Time::now(); // Set the timestamp to now for the transform
                                      // to work, because it tries to transform the
                                      // point at the time stamp of the input point
                target_speed = path_[point_idx_].pose.position.z;
                path_[point_idx_].pose.position.z = 0;
                tfBuffer_.transform(path_[point_idx_], target_point_, base_frame_,
                                    ros::Duration(0.1));
                path_[point_idx_].pose.position.z = target_speed;
                break;
              }
            }
          }

          if (!object_on_path) {
            // Calculate the steering angle
            ld_2 = ld_ * ld_;
            y_t = target_point_.pose.position.y;
            delta = atan2(2 * car_wheel_base_ * y_t, ld_2);
          }
        }

        control_msg_.drive.steering_angle = delta;
        control_msg_.drive.speed = target_speed;
        control_msg_.header.stamp = ros::Time::now();
        control_pub_.publish(control_msg_);

        last_p_idx_ = point_idx_;
        last_dist_ = distance_;
        if (point_idx_ == path_.size()) {
          n_laps_--;
          if (n_laps_ > 0) {
            point_idx_ = 0;
          } else {
            ROS_INFO("Reached final point");            
            control_msg_.drive.steering_angle = 0;
            control_msg_.drive.speed = 0;
            control_msg_.header.stamp = ros::Time::now();
            control_pub_.publish(control_msg_);
            got_path_ = false;
            point_idx_ = 0;            
          }
        }
        lookahead_p.point = path_[point_idx_].pose.position;
        lookahead_p.header = path_[point_idx_].header;
        l_point_pub_.publish(lookahead_p); // Publish the lookahead point
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
  delete ros_rate_;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit");
  vec_control::PurePursuit pp_node;
}