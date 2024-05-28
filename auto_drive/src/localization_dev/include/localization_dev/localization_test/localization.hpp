/**
 * @file localization.hpp
 * @brief Header file for the Localization class
 * @author kousei
 * @date 2024-05-29
 */

#ifndef LOCALIZATION_DEV__LOCALIZATION_HPP_
#define LOCALIZATION_DEV__LOCALIZATION_HPP_

#include "localization_dev/visibility_control.h"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace localization_dev {

/// @brief A class to estimate the robot's pose.
/// @details This class estimates the robot's pose by fusing the noisy odometry
/// and the laser scan data. The estimated pose is published as a Pose message.
/// x,y are the position of the robot, and z is the orientation of the robot.
/// The noisy odometry is received from the /noisy_odom topic, the laser scan
/// data is received from the /scan topic, and the map is received from the /map
/// topic and the point cloud is received from the /mapped_pc2 topic.
class Localization : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit Localization(const rclcpp::NodeOptions &options);
  virtual ~Localization();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      localization_switch_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr estimated_odom_pub;

  geometry_msgs::msg::Pose estimated_pose;
  sensor_msgs::msg::LaserScan scan;
  nav_msgs::msg::Odometry noisy_odom;
  nav_msgs::msg::OccupancyGrid map;
  sensor_msgs::msg::PointCloud2 pc;

  std::mutex mutex_;

  /// @brief Estimate the robot's pose.
  /// @return The estimated pose.
  /// @details This function estimates the robot's pose by fusing the noisy
  /// odometry and the laser scan data.
  geometry_msgs::msg::Pose estimate_pose();

  /// @brief Callback function for the localization switch.
  /// @param msg
  /// @return void
  /// @details when the localization switch on simulator is pushed, this
  /// function will be called.
  void localization_switch_callback(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__LOCALIZATION_HPP_