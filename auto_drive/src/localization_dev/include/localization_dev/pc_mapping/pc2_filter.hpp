#ifndef LOCALIZATION_DEV__PC2_FILTER_HPP_
#define LOCALIZATION_DEV__PC2_FILTER_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace localization_dev {
class Pc2Filter : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit Pc2Filter(const rclcpp::NodeOptions &options);
  virtual ~Pc2Filter();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pc2_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc2_pub;

  pcl::PointCloud<pcl::PointXYZ> filtered_pc2;
  void filter_pc2(pcl::PointCloud<pcl::PointXYZ> cloud);

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::mutex mutex_;
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__PC2_FILTER_HPP_