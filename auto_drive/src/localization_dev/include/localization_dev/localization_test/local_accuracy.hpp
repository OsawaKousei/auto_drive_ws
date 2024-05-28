/**
 * @file local_accuracy.hpp
 * @brief Header file for the LocalAccuracy class
 * @author kousei
 * @date 2024-05-29
*/

#ifndef LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_
#define LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

namespace localization_dev
{
  /// @brief A class to calculate the accuracy of the localization.
  /// @details This class calculates the accuracy of the localization by comparing the estimated odometry with the real odometry.
  /// The accuracy is calculated as 1 / (1 + cbrt(dx^2 + dy^2 + dth^2)), where dx, dy, and dth are the differences between the estimated and real odometry.
  /// The accuracy is published as a Float64 message.
  /// The estimated odometry is received from the /estimated_odom topic, and the real odometry is received from the /odom topic.
  class LocalAccuracy : public rclcpp::Node
  {
  public:
    TUTORIAL_PUBLIC
    explicit LocalAccuracy(const rclcpp::NodeOptions &options);
    virtual ~LocalAccuracy();

  private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr estimated_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accuracy_pub;

    float accuracy;
    geometry_msgs::msg::Pose estimated_odom;
    nav_msgs::msg::Odometry real_odom;

    std::mutex mutex_;
  };

} // namespace localization_dev

#endif // LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_
