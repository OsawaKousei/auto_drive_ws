/**
 * @file pc2_mapping.cpp
 * @brief Implementation file for the Pc2Mapping class
 * @author kousei
 * @date 2024-05-29
*/
#include "localization_dev/pc_mapping/pc2_mapping.hpp"
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

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

#include "tf2/exceptions.h"
#include "tf2/tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace std::chrono_literals;

namespace localization_dev {

Pc2Mapping::Pc2Mapping(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pc2_mapping", options) {
  current_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "raw_pc2", 1, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        current_pc2_callback(msg);
      });

  combined_pc2_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("combined_pc2", 10);

  filtered_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "filtered_pc2", 1,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        filtered_pc2_callback(msg);
      });

  mapped_pc2_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_pc2", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Pc2Mapping::~Pc2Mapping() {}

void Pc2Mapping::current_pc2_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  mutex_.lock();

  // transform point cloud
  sensor_msgs::msg::PointCloud2 transformed_pc2;
  try {
    tf_buffer_->transform(*msg, transformed_pc2, "map");
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    // return; // skip this point cloud if transform error. It is likely that tf
    // publisher between map and base_footprint has not been initialized yet.
  }

  *msg = transformed_pc2;

  // combine point cloud
  if (combined_pc2.data.size() == 0) {
    combined_pc2 = *msg;
  } else {
    // TODO: combine point cloud without using pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *msg_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(combined_pc2, *combined_cloud);
    *combined_cloud += *msg_cloud;
    pcl::toROSMsg(*combined_cloud, combined_pc2);
  }

  combined_pc2_pub->publish(combined_pc2);
  mutex_.unlock();
};

void Pc2Mapping::filtered_pc2_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  mutex_.lock();
  // publish filtered point cloud as mapped point cloud
  mapped_pc2 = *msg;
  mapped_pc2_pub->publish(mapped_pc2);
  mutex_.unlock();
};

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Pc2Mapping)