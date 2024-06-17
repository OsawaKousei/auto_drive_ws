#include <rclcpp/rclcpp.hpp>

#include "local_path/path_publisher.hpp"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    const auto path_publisher =
        std::make_shared<local_path::PathPublisher>(rclcpp::NodeOptions());
    exec.add_node(path_publisher);
    
    exec.spin();
    rclcpp::shutdown();
}