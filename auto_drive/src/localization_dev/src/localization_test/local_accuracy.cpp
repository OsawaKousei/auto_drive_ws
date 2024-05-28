/**
 * @file local_accuracy.cpp
 * @brief Implementation file for the LocalAccuracy class
 * @author kousei
 * @date 2024-05-29
 */

#include "localization_dev/localization_test/local_accuracy.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error "undefined reference to 'tf2::fromMsg(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'"
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR

using namespace std::chrono_literals;

namespace localization_dev {
LocalAccuracy::LocalAccuracy(const rclcpp::NodeOptions &options)
    : rclcpp::Node("local_accuracy", options) {
  auto estimated_odom_callback =
      [this](const geometry_msgs::msg::Pose msg) -> void {
    mutex_.lock();
    this->estimated_odom = msg;
    double yaw, pitch, roll;
    tf2::getEulerYPR(this->real_odom.pose.pose.orientation, yaw, pitch, roll);

    // Calculate the accuracy.
    this->accuracy =
        1 / (1 + cbrt(pow(this->estimated_odom.position.x -
                              this->real_odom.pose.pose.position.x,
                          2) +
                      pow(this->estimated_odom.position.y -
                              this->real_odom.pose.pose.position.y,
                          2) +
                      pow(this->estimated_odom.position.z - yaw, 2)));

    // Publish the accuracy.
    auto accuracy_msg = std_msgs::msg::Float64();
    accuracy_msg.data = this->accuracy;
    this->accuracy_pub->publish(accuracy_msg);
    std::cout << "Real pose: x: " << real_odom.pose.pose.position.x
              << ", y: " << real_odom.pose.pose.position.y << ", th:" << yaw
              << std::endl;
    std::cout << "Estimate Accuracy: " << this->accuracy << std::endl;
    mutex_.unlock();
  };

  this->estimated_odom_sub =
      this->create_subscription<geometry_msgs::msg::Pose>(
          "estimated_odom", 10, estimated_odom_callback);

  this->real_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry &msg) -> void {
        mutex_.lock();
        this->real_odom = msg;
        mutex_.unlock();
      });

  this->accuracy_pub =
      this->create_publisher<std_msgs::msg::Float64>("accuracy", 10);
}

LocalAccuracy::~LocalAccuracy() {}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::LocalAccuracy)
