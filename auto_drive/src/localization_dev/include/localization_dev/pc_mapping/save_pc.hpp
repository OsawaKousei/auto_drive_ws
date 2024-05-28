#ifndef LOCALIZATION_DEV__SAVE_PC_HPP_
#define LOCALIZATION_DEV__SAVE_PC_HPP_

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

class SavePc : public rclcpp::Node
{
public:
    TUTORIAL_PUBLIC
    explicit SavePc(const rclcpp::NodeOptions & options);

    virtual ~SavePc();
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr current_map_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pc_mapping_switch_sub;

    pcl::PointCloud<pcl::PointXYZ> current_map_pc;

    void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void switch_callback(const std_msgs::msg::String::SharedPtr msg);
    void save_pc(pcl::PointCloud<pcl::PointXYZ> cloud);

    //declare parameter
    std::string map_dir;

    std::mutex mutex_;
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__SAVE_PC_HPP_