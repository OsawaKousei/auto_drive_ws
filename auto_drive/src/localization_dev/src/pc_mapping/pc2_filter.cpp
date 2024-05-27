#include "localization_dev/pc_mapping/pc2_filter.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std::chrono_literals;


namespace localization_dev
{

Pc2Filter::Pc2Filter(const rclcpp::NodeOptions & options)
: rclcpp::Node("pc2_filter", options)
{
    auto topic_callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        // convert PointCloud2 to PointXYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",msg->height,msg->width);

        // PassThrough Filter
        filter_pc2(*cloud);

        // publish filtered point cloud
        sensor_msgs::msg::PointCloud2 filtered_pc2_msg;
        pcl::toROSMsg(filtered_pc2, filtered_pc2_msg);
        filtered_pc2_msg.header = msg->header;
        filtered_pc2_pub->publish(filtered_pc2_msg);
    };

    raw_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "raw_pc2", 10, topic_callback);

    filtered_pc2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pc2", 10);
}

// デストラクタ
Pc2Filter::~Pc2Filter()
{
}

// PassThrough Filter
void Pc2Filter::filter_pc2(pcl::PointCloud<pcl::PointXYZ> cloud){
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud)); // Convert to ConstPtr
    pass.setInputCloud(cloudPtr); // Use ConstPtr
    pass.setFilterFieldName("y");  // x axis
    // extract point cloud between 1.0 and 3.0 m
    pass.setFilterLimits(1.0,3.0);
    // pass.setFilterLimitsNegative (true);   // extract range reverse
    pass.filter(filtered_pc2);
};

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Pc2Filter)