#include <rclcpp/rclcpp.hpp>

#include "local_path/path_publisher.hpp"
#include "local_path/local_planner.hpp"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    const auto path_publisher =
        std::make_shared<local_path::PathPublisher>(rclcpp::NodeOptions());
    exec.add_node(path_publisher);

    const auto local_planner =
        std::make_shared<local_path::LocalPlanner>(rclcpp::NodeOptions());
    exec.add_node(local_planner);
    
    exec.spin();
    rclcpp::shutdown();
}