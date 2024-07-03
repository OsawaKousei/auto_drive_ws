/**
 * @file map_publisher.hpp
 * @brief Header file for the MapPublisher class
 * @author kousei
 * @date 2024-05-29
 */

#ifndef LOCALIZATION_DEV__MAP_PUBLISHER_HPP_
#define LOCALIZATION_DEV__MAP_PUBLISHER_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace localization_dev {
/// @brief A class to publish the map as a PointCloud2 message.
/// @details This class reads the map from a pcd file and publishes it as a
/// PointCloud2 message. The map directory and map name are set as parameters.
/// The map is published as a PointCloud2 message on the /mapped_pc2 topic.
class MapPublisher : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit MapPublisher(const rclcpp::NodeOptions &options);
  virtual ~MapPublisher();

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string map_dir;
  std::string map_name;
  std::mutex mutex_;

  void timer_callback();
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__MAP_PUBLISHER_HPP_
