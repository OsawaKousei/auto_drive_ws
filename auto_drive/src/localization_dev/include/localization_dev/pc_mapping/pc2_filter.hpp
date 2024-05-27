#ifndef LOCALIZATION_DEV__PC2_FILTER_HPP_
#define LOCALIZATION_DEV__PC2_FILTER_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace localization_dev
{

class Pc2Filter : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit Pc2Filter(const rclcpp::NodeOptions & options);

  virtual ~Pc2Filter();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pc2_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc2_pub;
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__PC2_FILTER_HPP_