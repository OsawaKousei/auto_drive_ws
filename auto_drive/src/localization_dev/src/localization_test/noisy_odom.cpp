/**
 * @file noisy_odom.cpp
 * @brief Implementation file for the NoisyOdom class
 * @author kousei
 * @date 2024-05-29
 */

#include "localization_dev/localization_test/noisy_odom.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace localization_dev {

NoisyOdom::NoisyOdom(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odom", options) {
  // Define random generator with Gaussian distribution
  this->xy_dist = std::normal_distribution<double>(xy_mean, xy_stddev);
  this->th_dist = std::normal_distribution<double>(th_mean, th_stddev);

  auto real_odom_callback = [this](const nav_msgs::msg::Odometry &msg) -> void {
    mutex_.lock();

    geometry_msgs::msg::Pose noisy_odom = msg.pose.pose;
    noisy_odom.position.x += this->xy_dist(this->generator);
    noisy_odom.position.y += this->xy_dist(this->generator);
    noisy_odom.position.z += this->xy_dist(this->generator);
    noisy_odom.orientation.x += this->th_dist(this->generator);
    noisy_odom.orientation.y += this->th_dist(this->generator);
    noisy_odom.orientation.z += this->th_dist(this->generator);
    noisy_odom.orientation.w += this->th_dist(this->generator);

    this->noisy_odom_pub->publish(noisy_odom);

    mutex_.unlock();
  };

  this->real_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, real_odom_callback);

  this->noisy_odom_pub =
      this->create_publisher<geometry_msgs::msg::Pose>("noisy_odom", 10);
}

NoisyOdom::~NoisyOdom() {}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::NoisyOdom)
