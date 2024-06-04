#ifndef LOCALIZATION_BYNAV2__ODOM_MODIFIER_HPP_
#define LOCALIZATION_BYNAV2__ODOM_MODIFIER_HPP_

#include "localization_bynav2/visibility_control.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace localization_bynav2 {

class OdomModifier : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit OdomModifier(const rclcpp::NodeOptions &options);
  virtual ~OdomModifier();

private:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr noisy_odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr odom_pub;

  geometry_msgs::msg::Point modified_odom;
  geometry_msgs::msg::Point error;

  std::mutex mutex_;
};

} // namespace localization_bynav2

#endif // LOCALIZATION_BYNAV2__ODOM_MODIFIER_HPP_