#include "localization_dev/localization_test/odom_modifier.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error "undefined reference to 'tf2::fromMsg(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'"
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR

using namespace std::chrono_literals;

namespace localization_dev {
OdomModifier::OdomModifier(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odom_modifier", options) {

    auto noisy_odom_callback = [this](const geometry_msgs::msg::Pose::SharedPtr msg) -> void {
        mutex_.lock();
        this->modified_odom.position.x = msg->position.x + this->error.position.x;
        this->modified_odom.position.y = msg->position.y + this->error.position.y;
        this->modified_odom.position.z = msg->position.z + this->error.position.z;
        this->odom_pub->publish(this->modified_odom);
        mutex_.unlock();
    };

    auto estimated_odom_callback = [this](const geometry_msgs::msg::Pose::SharedPtr msg) -> void {
        mutex_.lock();
        this->error.position.x = this->modified_odom.position.x - msg->position.x;
        this->error.position.y = this->modified_odom.position.y - msg->position.y;
        this->error.position.z = this->modified_odom.position.z - msg->position.z;
        mutex_.unlock();
    };

    this->noisy_odom_sub =
        this->create_subscription<geometry_msgs::msg::Pose>(
            "noisy_odom", 1, noisy_odom_callback);

    this->estimated_odom_sub =
        this->create_subscription<geometry_msgs::msg::Pose>(
            "estimated_odom", 1, estimated_odom_callback);

    this->odom_pub =
        this->create_publisher<geometry_msgs::msg::Pose>("modified_odom", 1);
}

OdomModifier::~OdomModifier() {}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::OdomModifier)
