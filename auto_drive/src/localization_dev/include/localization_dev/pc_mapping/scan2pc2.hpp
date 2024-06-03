/**
 * @file scan2pc2.hpp
 * @brief Header file for the Scan2pc2 class
 * @author kousei
 * @date 2024-05-29
*/
#ifndef LOCALIZATION_DEV__SCAN2PC2_HPP_
#define LOCALIZATION_DEV__SCAN2PC2_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace localization_dev {
/// @brief A class to convert the LaserScan data to the PointCloud2 data.
/// @details This class converts the LaserScan data to the PointCloud2 data
/// The raw data is received from the "scan" topic.
/// The converted data is published to the "raw_pc2" topic.
class Scan2pc2 : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit Scan2pc2(const rclcpp::NodeOptions &options);

  virtual ~Scan2pc2();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub;

  void scan_callback(const sensor_msgs::msg::LaserScan &scan);

  std::mutex mutex_;
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__SCAN2PC2_HPP_