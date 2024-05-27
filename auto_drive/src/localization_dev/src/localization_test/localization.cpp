#include "localization_dev/localization_test/localization.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <memory>
#include <iostream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error "undefined reference to 'tf2::fromMsg(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'"
#include <tf2/utils.h> //getEulerYPR
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;


namespace localization_dev
{

Localization::Localization(const rclcpp::NodeOptions & options)
: rclcpp::Node("localization", options)
{
    // Create a callback function for when messages are received.
    auto localization_switch_callback =
        [this](const std_msgs::msg::String &msg) -> void
        {
            // Store the localization switch message.
            if(msg.data == "localize")
            {   
                std::cout << "localize function called" << std::endl;
                this->estimate_pose();

                // Publish the estimated odometry.
                this->estimated_odom_pub->publish(this->estimated_pose);
            }
        };
    
    auto noisy_odom_callback =
        [this](const nav_msgs::msg::Odometry &msg) -> void
        {
        // Store the noisy odometry message.
        this->noisy_odom = msg;
        };
    
    auto scan_callback =
        [this](const sensor_msgs::msg::LaserScan &msg) -> void
        {
        // Store the scan message.
        this->scan = msg;
        };

    // Create a subscription to the localization switch.
    this->localization_switch_sub = this->create_subscription<std_msgs::msg::String>(
        "if_localize", 10, localization_switch_callback);
    
    // Create a subscription to the noisy odometry.
    this->noisy_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "noisy_odom", 10, noisy_odom_callback);
    
    // Create a subscription to the scan.
    this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, scan_callback);
    
    // Create a publisher for the estimated odometry.
    this->estimated_odom_pub = this->create_publisher<geometry_msgs::msg::Pose>("estimated_odom", 10);

    std::cout << "Localization node initialized" << std::endl;
}

// デストラクタ
Localization::~Localization()
{
}

sensor_msgs::msg::LaserScan::SharedPtr Localization::get_scan()
{
    return std::make_shared<sensor_msgs::msg::LaserScan>(this->scan);
}

nav_msgs::msg::Odometry::SharedPtr Localization::get_noisy_odom()
{
    return std::make_shared<nav_msgs::msg::Odometry>(this->noisy_odom);
}

geometry_msgs::msg::Pose Localization::estimate_pose()
{
    double yaw, pitch, roll;
    tf2::getEulerYPR(this->noisy_odom.pose.pose.orientation, yaw, pitch, roll); // quaternion to euler
    estimated_pose.position.x = this->noisy_odom.pose.pose.position.x;
    estimated_pose.position.y = this->noisy_odom.pose.pose.position.y;
    estimated_pose.position.z = yaw;

    std::cout << "Estimated pose: x: " << estimated_pose.position.x << ", y: " << estimated_pose.position.y << ", th:" << estimated_pose.position.z << std::endl;
    return estimated_pose;
}

}  // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::Localization)