/**
 * @file pc2_filter.cpp
 * @brief Implementation file for the Pc2Filter class
 * @author kousei
 * @date 2024-05-29
*/
#include "localization_dev/pc_mapping/pc2_filter.hpp"
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

using namespace std::chrono_literals;

namespace localization_dev {

Pc2Filter::Pc2Filter(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pc2_filter", options) {
  raw_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "combined_pc2", 1,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        topic_callback(msg);
      });

  filtered_pc2_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pc2", 10);
}

Pc2Filter::~Pc2Filter() {}

// PassThrough Filter
void Pc2Filter::filter_pc2(pcl::PointCloud<pcl::PointXYZ> cloud) {
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud.makeShared()); // Use ConstPtr
  pass.setFilterFieldName("z");           // axis
  // extract point cloud
  pass.setFilterLimits(-1.0, 1.0);
  pass.filter(filtered_pc2);

  // Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(filtered_pc2.makeShared());
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.5);
  sor.setNegative(false);
  sor.filter(filtered_pc2);

  // Voxel Grid: pattern 1
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(filtered_pc2.makeShared());
  float leaf_size_ = 0.05;
  // set the leaf size (x, y, z)
  voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // apply the filter to dereferenced cloudVoxel
  voxelGrid.filter(filtered_pc2);
};

void Pc2Filter::topic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  mutex_.lock();
  // convert PointCloud2 to PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // PassThrough Filter
  filter_pc2(*cloud);

  // publish filtered point cloud
  pcl::toROSMsg(filtered_pc2, *msg);
  filtered_pc2_pub->publish(*msg);
  mutex_.unlock();
};

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Pc2Filter)