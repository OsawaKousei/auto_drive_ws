/**
 * @file localization.cpp
 * @brief Implementation file for the Localization class
 * @author kousei
 * @date 2024-05-29
 */

#include "localization_dev/localization_test/localization.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error "undefined reference to 'tf2::fromMsg(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'"
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR

using namespace std::chrono_literals;

namespace localization_dev {

Localization::Localization(const rclcpp::NodeOptions &options)
    : rclcpp::Node("localization", options) {
  auto localization_switch_callback =
      [this](const std_msgs::msg::String &msg) -> void {
    mutex_.lock();
    if (msg.data == "localize") {
      std::cout << "localize function called" << std::endl;
      this->estimate_pose();
      this->estimated_odom_pub->publish(this->estimated_pose);
    }
    mutex_.unlock();
  };

  this->localization_switch_sub =
      this->create_subscription<std_msgs::msg::String>(
          "if_localize", 1, localization_switch_callback);

  this->noisy_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry &msg) -> void {
        mutex_.lock();
        this->odom = msg;
        mutex_.unlock();
      });

  this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1, [this](const sensor_msgs::msg::LaserScan &msg) -> void {
        mutex_.lock();
        this->scan = msg;
        mutex_.unlock();
      });

  this->map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, [this](const nav_msgs::msg::OccupancyGrid &msg) -> void {
        mutex_.lock();
        this->map = msg;
        mutex_.unlock();
      });

  this->pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "mapped_pc2", 1,
      [this](const sensor_msgs::msg::PointCloud2 &msg) -> void {
        mutex_.lock();
        this->pc = msg;
        mutex_.unlock();
      });

  this->estimated_odom_pub =
      this->create_publisher<geometry_msgs::msg::Pose>("estimated_odom", 10);

  std::cout << "Localization node initialized" << std::endl;
}

Localization::~Localization() {}

// TODO: Implement the estimate_pose function
geometry_msgs::msg::Pose Localization::estimate_pose() {
  double yaw, pitch, roll;
  tf2::getEulerYPR(this->odom.pose.pose.orientation, yaw, pitch,
                   roll); // quaternion to euler
  estimated_pose.position.x = this->odom.pose.pose.position.x;
  estimated_pose.position.y = this->odom.pose.pose.position.y;
  estimated_pose.position.z = yaw;
  std::cout << "Estimated pose: x: " << estimated_pose.position.x
            << ", y: " << estimated_pose.position.y
            << ", th:" << estimated_pose.position.z << std::endl;
  return estimated_pose;
}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Localization)
