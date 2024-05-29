/**
 * @file localization_test_node.cpp
 * @brief Node for localization test
 * @author kousei
 * @date 2024-05-29
 */

#include <rclcpp/rclcpp.hpp>

#include "nav2_localization/nav2_accuracy.hpp"
#include "nav2_localization/noisy_odom.hpp"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto nav2_accuracy =
      std::make_shared<nav2_localization::Nav2Accuracy>(rclcpp::NodeOptions());
  exec.add_node(nav2_accuracy);

  const auto noisy_odom =
      std::make_shared<nav2_localization::NoisyOdom>(rclcpp::NodeOptions());
  exec.add_node(noisy_odom);

  exec.spin();
  rclcpp::shutdown();
}