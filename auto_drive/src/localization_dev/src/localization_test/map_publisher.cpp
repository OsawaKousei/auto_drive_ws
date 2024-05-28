/**
 * @file map_publisher.cpp
 * @brief Implementation file for the MapPublisher class
 * @author kousei
 * @date 2024-05-29
 */

#include "localization_dev/localization_test/map_publisher.hpp"
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

namespace localization_dev {

MapPublisher::MapPublisher(const rclcpp::NodeOptions &options)
    : rclcpp::Node("map_publisher", options) {
  // Get the map directory and map name from the parameters
  declare_parameter("map_dir", "default");
  declare_parameter("map_name", "default");
  get_parameter("map_dir", map_dir);
  get_parameter("map_name", map_name);
  std::cout << "map_dir: " << map_dir << std::endl;
  std::cout << "map_name: " << map_name << std::endl;

  map_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_pc2", 10);
  timer_ = this->create_wall_timer(
      2000ms, std::bind(&MapPublisher::timer_callback, this));
}

MapPublisher::~MapPublisher() {}

void MapPublisher::timer_callback() {
  mutex_.lock();

  // read pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    pcl::io::loadPCDFile<pcl::PointXYZ>(map_dir + "/" + map_name, *cloud);
  } catch (std::runtime_error e) {
    std::cerr << "Error in reading pcd file: " << e.what() << std::endl;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = this->now();
  map_pub->publish(msg);

  mutex_.unlock();
}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::MapPublisher)
