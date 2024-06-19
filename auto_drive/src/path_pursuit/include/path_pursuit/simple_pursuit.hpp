#ifndef PATH_PURSUIT__SIMPLE_PURSUIT_HPP_
#define PATH_PURSUIT__SIMPLE_PURSUIT_HPP_

#include "path_pursuit/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "pid.hpp"

namespace path_pursuit {

class SimplePursuit : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit SimplePursuit(const rclcpp::NodeOptions &options);
  virtual ~SimplePursuit();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

  nav_msgs::msg::Path path_;
  std::string path_dir_;
  PID pid_x = PID(0.0, 0.0, 0.0);
  PID pid_y = PID(0.0, 0.0, 0.0);

  double kp, ki, kd;
  double ctrl_priod_;
  double distance_threshold_;
  double threshold_modify_rate_;
  double modified_threshold_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::mutex mutex_;
};

} // namespace path_pursuit

#endif // PATH_PURSUIT__SIMPLE_PURSUIT_HPP_
