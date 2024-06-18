#ifndef LOCAL_PATH__PATH_PUBLISHER_HPP_
#define LOCAL_PATH__PATH_PUBLISHER_HPP_

#include "local_path/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace local_path {

class PathPublisher : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit PathPublisher(const rclcpp::NodeOptions &options);
  virtual ~PathPublisher();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  nav_msgs::msg::Path path_;

  double distance_threshold_;
  int next_idx_;
  std::string path_dir_;

  std::mutex mutex_;
};

} // namespace local_path

#endif // LOCAL_PATH__PATH_PUBLISHER_HPP_
