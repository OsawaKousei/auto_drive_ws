#ifndef LOCALIZATION_DEV__ODOM_MODIFIER_HPP_
#define LOCALIZATION_DEV__ODOM_MODIFIER_HPP_

#include "localization_dev/visibility_control.h"
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace localization_dev {

class OdomModifier : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit OdomModifier(const rclcpp::NodeOptions &options);
  virtual ~OdomModifier();

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr noisy_odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr estimated_odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr odom_pub;

  geometry_msgs::msg::Pose modified_odom;
  geometry_msgs::msg::Pose error;

  std::mutex mutex_;
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__ODOM_MODIFIER_HPP_