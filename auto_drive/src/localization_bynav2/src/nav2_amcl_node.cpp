/**
 * @file localization_test_node.cpp
 * @brief Node for localization test
 * @author kousei
 * @date 2024-05-29
 */

#include <rclcpp/rclcpp.hpp>

#include "localization_bynav2/nav2_accuracy.hpp"
#include "localization_bynav2/noisy_odom.hpp"
#include "localization_bynav2/odom_modifier.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto nav2_accuracy =
      std::make_shared<localization_bynav2::Nav2Accuracy>(
          rclcpp::NodeOptions());
  exec.add_node(nav2_accuracy);

  const auto noisy_odom =
      std::make_shared<localization_bynav2::NoisyOdom>(rclcpp::NodeOptions());
  exec.add_node(noisy_odom);

  const auto odom_modifier =
      std::make_shared<localization_bynav2::OdomModifier>(rclcpp::NodeOptions());
  exec.add_node(odom_modifier);

  exec.spin();
  rclcpp::shutdown();
}