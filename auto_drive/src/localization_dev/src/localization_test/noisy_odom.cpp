/**
 * @file noisy_odom.cpp
 * @brief Implementation file for the NoisyOdom class
 * @author kousei
 * @date 2024-05-29
*/

#include "localization_dev/localization_test/noisy_odom.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;


namespace localization_dev
{

NoisyOdom::NoisyOdom(const rclcpp::NodeOptions & options)
: rclcpp::Node("noisy_odom", options)
{
    // Define random generator with Gaussian distribution
    
    this->xy_dist = std::normal_distribution<double>(xy_mean, xy_stddev);;
    this->th_dist = std::normal_distribution<double>(th_mean, th_stddev);

    // Create a callback function for when messages are received.
    auto real_odom_callback =
        [this](const nav_msgs::msg::Odometry &msg) -> void
        {
        // Store the real odometry message.
        nav_msgs::msg::Odometry noisy_odom = msg;
        noisy_odom.pose.pose.position.x += this->xy_dist(this->generator);
        noisy_odom.pose.pose.position.y += this->xy_dist(this->generator);
        noisy_odom.pose.pose.position.z += this->xy_dist(this->generator);
        noisy_odom.pose.pose.orientation.x += this->th_dist(this->generator);
        noisy_odom.pose.pose.orientation.y += this->th_dist(this->generator);
        noisy_odom.pose.pose.orientation.z += this->th_dist(this->generator);
        noisy_odom.pose.pose.orientation.w += this->th_dist(this->generator);
        this->noisy_odom_pub->publish(noisy_odom);
    };
    
    // Create a subscription to the real odometry.
    this->real_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, real_odom_callback);
    
    // Create a publisher for the noisy odometry.
    this->noisy_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("noisy_odom", 10);
}

// デストラクタ
NoisyOdom::~NoisyOdom()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::NoisyOdom)