#include "localization_dev/localization.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>

using namespace std::chrono_literals;


namespace localization_dev
{

Localization::Localization(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_accuracy", options)
{
    
}

// デストラクタ
Localization::~Localization()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Localization)