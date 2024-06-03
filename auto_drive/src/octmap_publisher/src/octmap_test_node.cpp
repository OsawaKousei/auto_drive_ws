/**
 * @file octmap_test_node.cpp
 * @brief Publish a octmap data to the "octmap" topic
 * @author kousei
 * @date 2024-05-29
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rclcpp/qos.hpp" // Include the header file for QoS class

using namespace std::chrono_literals;

class OctmapTestNode : public rclcpp::Node {
public:
  OctmapTestNode() : Node("octmap_test_node") {

    octmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "octmap",
        rclcpp::QoS(10)
            .reliable()
            .transient_local()); // Create a publisher with QoS settings

    auto publish_msg_callback = [this]() -> void {
      auto message = nav_msgs::msg::OccupancyGrid();
      // create a scan message
      message.header.frame_id = "map";
      message.info.resolution = 0.1;
      message.info.width = 10;
      message.info.height = 10;
      message.info.origin.position.x = 0.0;
      message.info.origin.position.y = 0.0;
      message.info.origin.position.z = 0.0;
      message.info.origin.orientation.x = 0.0;
      message.info.origin.orientation.y = 0.0;
      message.info.origin.orientation.z = 0.0;
      message.info.origin.orientation.w = 1.0;
      message.data.resize(100);
      for (int i = 0; i < 100; i++) {
        if (i <= 50) {
          message.data[i] = 0;
        } else {
          message.data[i] = 100;
        }
      }

      octmap_pub->publish(message);
    };
    timer_ = this->create_wall_timer(500ms, publish_msg_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr octmap_pub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctmapTestNode>());
  rclcpp::shutdown();
  return 0;
}