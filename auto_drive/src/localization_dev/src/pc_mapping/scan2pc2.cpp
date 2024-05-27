#include "localization_dev/pc_mapping/scan2pc2.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;


namespace localization_dev
{

Scan2pc2::Scan2pc2(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_accuracy", options)
{
    
}

// デストラクタ
Scan2pc2::~Scan2pc2()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Scan2pc2)