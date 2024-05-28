#include <rclcpp/rclcpp.hpp>

#include "localization_dev/localization_test/local_accuracy.hpp"
#include "localization_dev/localization_test/localization.hpp"
#include "localization_dev/localization_test/noisy_odom.hpp"
#include "localization_dev/localization_test/map_publisher.hpp"
#include "localization_dev/pc_mapping/odom_tf.hpp"


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
    const auto map_publisher = std::make_shared<localization_dev::MapPublisher>(rclcpp::NodeOptions());
    exec.add_node(map_publisher);
    const auto odom_tf = std::make_shared<localization_dev::OdomTf>(rclcpp::NodeOptions());
    exec.add_node(odom_tf);
    exec.spin();
    rclcpp::shutdown();
}