#include <rclcpp/rclcpp.hpp>

#include <local_path/f7_sim_node.hpp>
#include <local_path/noisy_odom_node.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto _f7_sim_node =
      std::make_shared<f7_sim::F7_SIMNode>(rclcpp::NodeOptions());
  exec.add_node(_f7_sim_node);

  const auto _noisy_odom_node =
      std::make_shared<f7_sim::NoisyOdomNode>(rclcpp::NodeOptions());
  exec.add_node(_noisy_odom_node);

  exec.spin();
  rclcpp::shutdown();
}