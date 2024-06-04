#include <rclcpp/rclcpp.hpp>

#include "f7_sim/f7_sim.hpp"
#include "f7_sim/noisy_odom.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto f7_sim = std::make_shared<f7_sim::F7Sim>(rclcpp::NodeOptions());
  exec.add_node(f7_sim);

  const auto noisy_odom =
    std::make_shared<f7_sim::NoisyOdom>(rclcpp::NodeOptions());
  exec.add_node(noisy_odom);

  exec.spin();
  rclcpp::shutdown();
}