#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <fstream>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "local_path/path_publisher.hpp"

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

  std::mutex mutex_;

  void path_callback(const nav_msgs::msg::Path::SharedPtr path) {
    mutex_.lock();

    auto corner_pc2 = path2pc2(*path);
    corner_pc2->header.frame_id = "map";
    corner_pc2->header.stamp = this->now();
    corner_publisher_->publish(*corner_pc2);

    mutex_.unlock();
  }

  sensor_msgs::msg::PointCloud2::SharedPtr path2pc2(nav_msgs::msg::Path path){
    sensor_msgs::msg::PointCloud2 pc2;

    pc2.header = path.header;
    pc2.height = 1;
    pc2.width = path.poses.size();
    pc2.fields.resize(3);
    pc2.fields[0].name = "x";
    pc2.fields[0].offset = 0;
    pc2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[0].count = 1;
    pc2.fields[1].name = "y";
    pc2.fields[1].offset = 4;
    pc2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[1].count = 1;
    pc2.fields[2].name = "z";
    pc2.fields[2].offset = 8;
    pc2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[2].count = 1;
    pc2.is_bigendian = false;
    pc2.point_step = 12;
    pc2.row_step = 12 * path.poses.size();
    pc2.is_dense = true;
    pc2.data.resize(pc2.point_step * pc2.width);
    float *data = reinterpret_cast<float *>(pc2.data.data());
    for (int i = 0; i < path.poses.size(); i++) {
      data[i * 3] = path.poses[i].pose.position.x;
      data[i * 3 + 1] = path.poses[i].pose.position.y;
      data[i * 3 + 2] = 0.0;
    }

    return std::make_shared<sensor_msgs::msg::PointCloud2>(pc2);
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