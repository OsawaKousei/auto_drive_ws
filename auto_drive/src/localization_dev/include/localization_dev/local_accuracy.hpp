#ifndef LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_
#define LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>


namespace localization_dev
{

class LocalAccuracy : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit LocalAccuracy(const rclcpp::NodeOptions & options);

  virtual ~LocalAccuracy();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accuracy_pub;

    float accuracy;

};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_