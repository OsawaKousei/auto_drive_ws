/**
 * @file pc2octmap.hpp
 * @brief Header file for the Pc2octmap class
 * @author kousei
 * @date 2024-05-29
 */
#ifndef LOCALIZATION_DEV__PC2OCTMAP_HPP_
#define LOCALIZATION_DEV__PC2OCTMAP_HPP_

#include "localization_dev/visibility_control.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace localization_dev {
/// @brief A class to map the PointCloud2 data to the OccupancyGrid data.
/// @details This class maps the PointCloud2 data to the OccupancyGrid data
/// the point cloud data is received from the "mapped_pc2" topic.
/// The OccupancyGrid data is published to the "map" topic.
class Pc2octmap : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit Pc2octmap(const rclcpp::NodeOptions &options);

  virtual ~Pc2octmap();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::mutex mutex_;
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__PC2OCTMAP_HPP_