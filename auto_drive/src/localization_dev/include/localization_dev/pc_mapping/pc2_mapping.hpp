/**
 * @file pc2_mapping.hpp
 * @brief Header file for the Pc2Mapping class
 * @author kousei
 * @date 2024-05-29
 */
#ifndef LOCALIZATION_DEV__PC2_MAPPING_HPP_
#define LOCALIZATION_DEV__PC2_MAPPING_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "localization_dev/visibility_control.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace localization_dev {
/// @brief A class to map the PointCloud2 data.
/// @details This class maps the PointCloud2 data
/// this class combines current map and new point cloud data which is got from
/// the lidar sensor. The raw data is received from the "raw_pc2" topic. The
/// combined data is published to the "combined_pc2" topic. this class get the
/// filtered point cloud data and publish it as mapped point cloud data. The
/// filtered data is received from the "filtered_pc2" topic. The mapped data is
/// published to the "mapped_pc2" topic.

class Pc2Mapping : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit Pc2Mapping(const rclcpp::NodeOptions &options);

  virtual ~Pc2Mapping();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      current_pc2_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc2_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      filtered_pc2_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapped_pc2_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  sensor_msgs::msg::PointCloud2 combined_pc2;
  sensor_msgs::msg::PointCloud2 mapped_pc2;

  void current_pc2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void
  filtered_pc2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::mutex mutex_;
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__PC2_MAPPING_HPP_