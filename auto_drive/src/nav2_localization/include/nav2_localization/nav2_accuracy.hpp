/**
 * @file nav2_accuracy.hpp
 * @brief Header file for the Nav2Accuracy class
 * @author kousei
 * @date 2024-05-29
*/

#ifndef NAV2_LOCALIZATION__NAV2_ACCURACY_HPP_
#define NAV2_LOCALIZATION__NAV2_ACCURACY_HPP_

#include "nav2_localization/visibility_control.h"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace nav2_localization {
/// @brief A class to calculate the accuracy of the navigation system.
/// @details This class calculates the accuracy of the navigation system by
/// comparing the estimated odometry and the real odometry. The estimated odometry
/// is obtained from the /estimated_odom topic, and the real odometry is obtained
class Nav2Accuracy : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit Nav2Accuracy(const rclcpp::NodeOptions &options);
  virtual ~Nav2Accuracy();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_odom_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accuracy_pub;

  float accuracy;
  geometry_msgs::msg::PoseWithCovarianceStamped estimated_odom;
  nav_msgs::msg::Odometry real_odom;

  std::mutex mutex_;

  
  float get_yaw(const geometry_msgs::msg::Quaternion &q);
};

} // namespace nav2_localization

#endif // NAV2_LOCALIZATION__NAV2_ACCURACY_HPP_