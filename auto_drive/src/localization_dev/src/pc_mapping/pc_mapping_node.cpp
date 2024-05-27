#include <rclcpp/rclcpp.hpp>

#include "localization_dev/pc_mapping/pc2_mapping.hpp"
#include "localization_dev/pc_mapping/pc2_filter.hpp"
#include "localization_dev/pc_mapping/scan2pc2.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    const auto local_accuracy = std::make_shared<localization_dev::Pc2Mapping>(rclcpp::NodeOptions());
    exec.add_node(local_accuracy);
    const auto localization = std::make_shared<localization_dev::Pc2Filter>(rclcpp::NodeOptions());
    exec.add_node(localization);
    const auto noisy_odom = std::make_shared<localization_dev::Scan2pc2>(rclcpp::NodeOptions());
    exec.add_node(noisy_odom);
    exec.spin();
    rclcpp::shutdown();
}