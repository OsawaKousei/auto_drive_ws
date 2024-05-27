#ifndef LOCALIZATION_DEV__SCAN2PC2_HPP_
#define LOCALIZATION_DEV__SCAN2PC2_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


namespace localization_dev
{

class Scan2pc2 : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit Scan2pc2(const rclcpp::NodeOptions & options);

  virtual ~Scan2pc2();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub;

    void scan_callback(const sensor_msgs::msg::LaserScan &scan);

    std::mutex mutex_;
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__SCAN2PC2_HPP_