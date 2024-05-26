#ifndef LOCALIZATION_DEV__LOCALIZATION_HPP_
#define LOCALIZATION_DEV__LOCALIZATION_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>


namespace localization_dev
{

class Localization : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit Localization(const rclcpp::NodeOptions & options);

  virtual ~Localization();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_switch_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr estimated_odom_pub;

    geometry_msgs::msg::Pose estimated_pose;
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry noisy_odom;

    sensor_msgs::msg::LaserScan::SharedPtr get_scan();
    nav_msgs::msg::Odometry::SharedPtr get_noisy_odom();
    geometry_msgs::msg::Pose estimate_pose();
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__LOCALIZATION_HPP_