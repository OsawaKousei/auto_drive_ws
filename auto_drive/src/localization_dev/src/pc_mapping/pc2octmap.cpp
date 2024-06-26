/**
 * @file pc2octmap.cpp
 * @brief Implementation file for the Pc2octmap class
 * @author kousei
 * @date 2024-05-29
 */
#include "localization_dev/pc_mapping/pc2octmap.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std::chrono_literals;

namespace localization_dev {

Pc2octmap::Pc2octmap(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pc2octmap", options) {
  map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "mapped_pc2", 1,
      std::bind(&Pc2octmap::map_callback, this, std::placeholders::_1));
  map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).reliable().transient_local());
}

Pc2octmap::~Pc2octmap() {}

void Pc2octmap::map_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  mutex_.lock();

  // create the occupancy grid map
  nav_msgs::msg::OccupancyGrid map;
  map.header = msg->header;
  map.info.map_load_time = msg->header.stamp;
  map.info.resolution = 0.075;
  map.info.width = 150;
  map.info.height = 150;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);

  // innitialize the occupancy grid map
  for (int i = 0; i < map.info.width; i++) {
    for (int j = 0; j < map.info.height; j++) {
      map.data[i + j * map.info.width] = 0;
    }
  }

  // transrate the point cloud to the occupancy grid map
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  for (int i = 0; i < cloud.size(); i++) {
    int x =
        (int)((cloud[i].x - map.info.origin.position.x) / map.info.resolution);
    int y =
        (int)((cloud[i].y - map.info.origin.position.y) / map.info.resolution);

    map.data[x + y * map.info.width] = 100;
  }

  // publish the map
  map_pub->publish(map);

  mutex_.unlock();
}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Pc2octmap)