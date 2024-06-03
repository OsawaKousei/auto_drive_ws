/**
 * @file dammy_scan_node.cpp
 * @brief Publish a dammy scan data to the "scan" topic
 * @author kousei
 * @date 2024-05-29
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class DammyScanNode : public rclcpp::Node {
public:
  DammyScanNode() : Node("dammy_scan_node") {

    publisher_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    auto publish_msg_callback = [this]() -> void {
      auto message = sensor_msgs::msg::LaserScan();
      // create a scan message
      message.header.frame_id = "map";
      message.angle_min = -M_PI / 2;
      message.angle_max = M_PI / 2;
      message.angle_increment = M_PI / 2;
      message.time_increment = 0.1;
      message.scan_time = 0.1;
      message.range_min = 0.0;
      message.range_max = 10.0;
      message.ranges.resize(4);
      message.intensities.resize(4);
      message.ranges[0] = 1.0;
      message.intensities[0] = 1.0;
      message.ranges[1] = 1.0;
      message.intensities[1] = 1.0;
      message.ranges[2] = 1.0;
      message.intensities[2] = 1.0;
      message.ranges[3] = 1.0;
      message.intensities[3] = 1.0;

      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(500ms, publish_msg_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DammyScanNode>());
  rclcpp::shutdown();
  return 0;
}