/**
 * @file noisy_odom.cpp
 * @brief Implementation file for the NoisyOdom class
 * @author kousei
 * @date 2024-05-29
 */

#include "localization_bynav2/noisy_odom.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace localization_bynav2 {

NoisyOdom::NoisyOdom(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odom", options) {
  // Define random generator with Gaussian distribution
  this->xy_dist = std::normal_distribution<double>(xy_mean, xy_stddev);
  this->th_dist = std::normal_distribution<double>(th_mean, th_stddev);

  auto real_odom_callback = [this](const nav_msgs::msg::Odometry &msg) -> void {
    mutex_.lock();

    geometry_msgs::msg::Point noisy_odom;
    noisy_odom.x = msg.pose.pose.position.x;
    noisy_odom.y = msg.pose.pose.position.y;
    double yaw, pitch, roll;
    tf2::getEulerYPR(msg.pose.pose.orientation, yaw, pitch,
                   roll); // quaternion to euler
    noisy_odom.z = yaw;
    
    noisy_odom.x += this->xy_dist(this->generator);
    noisy_odom.y += this->xy_dist(this->generator);
    noisy_odom.z += this->xy_dist(this->generator);

    this->noisy_odom_pub->publish(noisy_odom);

    mutex_.unlock();
  };

  this->real_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "raw_odom", 10, real_odom_callback);

  this->noisy_odom_pub =
      this->create_publisher<geometry_msgs::msg::Point>("noisy_odom", 10);
}

NoisyOdom::~NoisyOdom() {}

} // namespace localization_bynav2

RCLCPP_COMPONENTS_REGISTER_NODE(localization_bynav2::NoisyOdom)
