#include <rclcpp/rclcpp.hpp>

#include "localization_dev/pc_mapping/odom_tf.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  const auto odom_tf =
      std::make_shared<localization_dev::OdomTf>(rclcpp::NodeOptions());
  exec.add_node(odom_tf);
  exec.spin();
  rclcpp::shutdown();
}