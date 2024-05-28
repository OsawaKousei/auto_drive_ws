#ifndef LOCALIZATION_DEV__MAP_PUBLISHER_HPP_
#define LOCALIZATION_DEV__MAP_PUBLISHER_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace localization_dev
{

class MapPublisher : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit MapPublisher(const rclcpp::NodeOptions & options);

  virtual ~MapPublisher();

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_dir;   
    std::string map_name;
    std::mutex mutex_;

    void timer_callback();
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__MAP_PUBLISHER_HPP_