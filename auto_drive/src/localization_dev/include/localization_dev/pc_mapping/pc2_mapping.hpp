#ifndef LOCALIZATION_DEV__PC2_MAPPING_HPP_
#define LOCALIZATION_DEV__PC2_MAPPING_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace localization_dev
{

class Pc2Mapping : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit Pc2Mapping(const rclcpp::NodeOptions & options);

  virtual ~Pc2Mapping();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr current_pc2_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapped_pc2_pub;
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__PC2_MAPPING_HPP_