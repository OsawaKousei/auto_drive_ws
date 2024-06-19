#ifndef LOCAL_PATH__LOCAL_PLANNER_HPP_
#define LOCAL_PATH__LOCAL_PLANNER_HPP_

#include "local_path/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace local_path {

class LocalPlanner : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit LocalPlanner(const rclcpp::NodeOptions &options);
  virtual ~LocalPlanner();

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Odometry odom_;

  std::vector<double> xs;
  std::vector<double> ys;

  int path_points_;
  int consider_points_;

  std::mutex mutex_;
};

} // namespace local_path

#endif // LOCAL_PATH__LOCAL_PLANNER_HPP_