/**
 * @file nav2_accuracy.cpp
 * @brief Implementation file for the Nav2Accuracy class
 * @author kousei
 * @date 2024-05-29
 */

#include "localization_dev/localization_test/nav2_accuracy.hpp"
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR

using namespace std::chrono_literals;

namespace localization_dev {

Nav2Accuracy::Nav2Accuracy(const rclcpp::NodeOptions &options)
 : rclcpp::Node("nav2_accuracy", options) {
  // create subscription to the estimated odometry
  estimated_odom_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "estimated_odom", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        mutex_.lock();
        accuracy = 1 / (1 + cbrt(pow(msg->pose.pose.position.x - real_odom.pose.pose.position.x, 2) +
                                    pow(msg->pose.pose.position.y - real_odom.pose.pose.position.y, 2) +
                                    pow(get_yaw(msg->pose.pose.orientation) - get_yaw(real_odom.pose.pose.orientation), 2)));

        auto accuracy_msg = std_msgs::msg::Float64();
        accuracy_msg.data = accuracy;
        accuracy_pub->publish(accuracy_msg);
        RCLCPP_INFO(this->get_logger(), "Real odom: x: %f, y: %f, th: %f", real_odom.pose.pose.position.x,
                real_odom.pose.pose.position.y, get_yaw(real_odom.pose.pose.orientation));
        RCLCPP_INFO(this->get_logger(), "Estimated pose: x: %f, y: %f, th: %f", msg->pose.pose.position.x,
                msg->pose.pose.position.y, get_yaw(msg->pose.pose.orientation));
        RCLCPP_INFO(this->get_logger(), "Estimate Accuracy: %f", accuracy);
        mutex_.unlock();
      });

  // create subscription to the real odometry
  real_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "raw_odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        mutex_.lock();
        real_odom = *msg;
        mutex_.unlock();
      });
  
  // create publisher for the accuracy
  accuracy_pub = this->create_publisher<std_msgs::msg::Float64>("accuracy", 10);
}

Nav2Accuracy::~Nav2Accuracy() {}

float Nav2Accuracy::get_yaw(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}
} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Nav2Accuracy)
