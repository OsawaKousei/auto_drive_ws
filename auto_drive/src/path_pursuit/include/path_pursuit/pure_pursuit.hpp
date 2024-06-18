#ifndef PATH_PURSUIT__PATH_PURSUIT_HPP_
#define PATH_PURSUIT__PATH_PURSUIT_HPP_

#include "path_pursuit/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "pid.hpp"

namespace path_pursuit {

class PurePursuit : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit PurePursuit(const rclcpp::NodeOptions &options);
  virtual ~PurePursuit();

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  nav_msgs::msg::Path path_;
  PID pid_x = PID(0.0, 0.0, 0.0);
  PID pid_y = PID(0.0, 0.0, 0.0);

  double kp, ki, kd;
  int target_idx_;
  double target_dist_;
  nav_msgs::msg::Odometry prev_odom_;
  double vel_;
  double ctrl_priod_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::mutex mutex_;
};

} // namespace path_pursuit

#endif // PATH_PURSUIT__PATH_PURSUIT_HPP_
