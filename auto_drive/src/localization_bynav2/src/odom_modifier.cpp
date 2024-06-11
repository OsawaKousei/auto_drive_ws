#include "localization_bynav2/odom_modifier.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace localization_bynav2 {
OdomModifier::OdomModifier(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odom_modifier", options) {

    auto noisy_odom_callback = [this](const geometry_msgs::msg::Point::SharedPtr msg) -> void {
        mutex_.lock();
        this->modified_odom.x = msg->x + this->error.x;
        this->modified_odom.y = msg->y + this->error.y;
        this->modified_odom.z = msg->z + this->error.z;
        this->odom_pub->publish(this->modified_odom);
        mutex_.unlock();
    };

    auto estimated_odom_callback = [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void {
        mutex_.lock();
        this->error.x = this->modified_odom.x - msg->pose.pose.position.x;
        this->error.y = this->modified_odom.y - msg->pose.pose.position.y;
        double yaw, pitch, roll;
        tf2::getEulerYPR(msg.get()->pose.pose.orientation, yaw, pitch,
                   roll); // quaternion to euler
        this->error.z = this->modified_odom.z - yaw;
        mutex_.unlock();
    };

    this->estimated_odom_sub =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, estimated_odom_callback);

    this->noisy_odom_sub =
        this->create_subscription<geometry_msgs::msg::Point>(
            "noisy_odom", 10, noisy_odom_callback);

    this->odom_pub =
      this->create_publisher<geometry_msgs::msg::Point>("modified_odom", 10);
}

OdomModifier::~OdomModifier() {}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_bynav2::OdomModifier)
