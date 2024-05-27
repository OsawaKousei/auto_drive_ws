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
: rclcpp::Node("local_accuracy", options)
{
    
}

// デストラクタ
Pc2Mapping::~Pc2Mapping()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Pc2Mapping)