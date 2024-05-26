#include <rclcpp/rclcpp.hpp>

#include "localization_dev/local_accuracy.hpp"
#include "localization_dev/localization.hpp"
#include "localization_dev/noisy_odom.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    const auto local_accuracy = std::make_shared<localization_dev::LocalAccuracy>(rclcpp::NodeOptions());
    exec.add_node(local_accuracy);
    const auto localization = std::make_shared<localization_dev::Localization>(rclcpp::NodeOptions());
    exec.add_node(localization);
    const auto noisy_odom = std::make_shared<localization_dev::NoisyOdom>(rclcpp::NodeOptions());
    exec.add_node(noisy_odom);
    exec.spin();
    rclcpp::shutdown();
}