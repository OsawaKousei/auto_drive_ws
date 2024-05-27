#include "localization_dev/pc_mapping/pc2_mapping.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;


namespace localization_dev
{

Pc2Mapping::Pc2Mapping(const rclcpp::NodeOptions & options)
: rclcpp::Node("pc2_mapping", options)
{
    auto current_pc2_callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        // combine point cloud
        if(mapped_pc2.data.size() == 0){
            mapped_pc2 = *msg;
        }else{
            mapped_pc2.data.insert(mapped_pc2.data.end(), msg->data.begin(), msg->data.end());
        }
        combined_pc2_pub->publish(mapped_pc2);
    };

    auto filtered_pc2_callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        // publish filtered point cloud as mapped point cloud
        mapped_pc2 = *msg;
        mapped_pc2_pub->publish(mapped_pc2);
    };

    current_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "current_pc2", 10, current_pc2_callback);

    combined_pc2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("combined_pc2", 10);

    filtered_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filtered_pc2", 10, filtered_pc2_callback);

    mapped_pc2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapped_pc2", 10);
}

// デストラクタ
Pc2Mapping::~Pc2Mapping()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Pc2Mapping)