#include "localization_dev/local_accuracy.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>

using namespace std::chrono_literals;


namespace localization_dev
{

LocalAccuracy::LocalAccuracy(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_accuracy", options)
{
    
}

// デストラクタ
LocalAccuracy::~LocalAccuracy()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::LocalAccuracy)