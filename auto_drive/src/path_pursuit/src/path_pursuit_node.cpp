#include <rclcpp/rclcpp.hpp>

#include "path_pursuit/pure_pursuit.hpp"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    const auto pure_pursuit =
        std::make_shared<path_pursuit::PurePursuit>(rclcpp::NodeOptions());
    exec.add_node(pure_pursuit);
    
    exec.spin();
    rclcpp::shutdown();
}