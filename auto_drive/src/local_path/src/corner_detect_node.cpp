#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <fstream>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include "local_path/path_publisher.hpp"
#include "util/pcp.hpp"
#include "util/rviz_util.hpp"

using namespace std::chrono_literals;
namespace local_path {

class CornerDetectNode : public rclcpp::Node {
public:
  CornerDetectNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("local_planner", options) {

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_path", 1,
        [this](const nav_msgs::msg::Path::SharedPtr path) { path_callback(path); });

    corner_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corner", 1);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  std::mutex mutex_;

  void path_callback(const nav_msgs::msg::Path::SharedPtr path) {
    mutex_.lock();

    auto corner_pc2 = pcp::PCConvert::path2pc2(*path);
    corner_pc2->header.frame_id = "map";
    corner_pc2->header.stamp = this->now();
    corner_publisher_->publish(*corner_pc2);

    pcp::PCFeatureDetection pc(corner_pc2);
    auto [x, y, pv] = pc.PCA();

    std::cout << "x: " << x << ", y: " << y << ", pv: " << pv << std::endl;

    // auto viz_marker = viz_marker::std_line_setter(std::make_tuple(0.0, 0.0), std::make_tuple(x, y));
    // viz_marker->header.frame_id = "map";
    // viz_marker->header.stamp = this->now();
    // marker_publisher_->publish(*viz_marker);

    mutex_.unlock();
  }

};
  
} // namespace local_path

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto coner_detect =
      std::make_shared<local_path::CornerDetectNode>(rclcpp::NodeOptions());
  exec.add_node(coner_detect);

  const auto path_publisher =
      std::make_shared<local_path::PathPublisher>(rclcpp::NodeOptions());
  exec.add_node(path_publisher);
  
  exec.spin();
  rclcpp::shutdown();
}