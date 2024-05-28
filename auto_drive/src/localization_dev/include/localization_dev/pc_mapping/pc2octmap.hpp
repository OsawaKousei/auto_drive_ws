#ifndef LOCALIZATION_DEV__PC2OCTMAP_HPP_
#define LOCALIZATION_DEV__PC2OCTMAP_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace localization_dev
{

class Pc2octmap : public rclcpp::Node
{
public:
    TUTORIAL_PUBLIC
    explicit Pc2octmap(const rclcpp::NodeOptions & options);

    virtual ~Pc2octmap();
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

    void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::mutex mutex_;
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__PC2OCTMAP_HPP_