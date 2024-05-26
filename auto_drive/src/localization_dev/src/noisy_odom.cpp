#include "localization_dev/noisy_odom.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>

using namespace std::chrono_literals;


namespace localization_dev
{

NoisyOdom::NoisyOdom(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_accuracy", options)
{
    
}

// デストラクタ
NoisyOdom::~NoisyOdom()
{
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::NoisyOdom)