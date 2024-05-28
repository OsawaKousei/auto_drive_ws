/**
 * @file odom_tf.hpp
 * @brief Header file for the OdomTf class
 * @author kousei
 * @date 2024-05-29
*/

#ifndef LOCALIZATION_DEV__ODOM_TF_HPP_
#define LOCALIZATION_DEV__ODOM_TF_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace localization_dev
{
    /// @brief A class to publish the odometry as a tf.
    /// @details This class publishes the tf between the odom and base_footprint frames.
    /// The odometry is received from the /odom topic.
    class OdomTf : public rclcpp::Node
    {
    public:
        TUTORIAL_PUBLIC
        explicit OdomTf(const rclcpp::NodeOptions &options);
        virtual ~OdomTf();

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        geometry_msgs::msg::TransformStamped odom_tf;
        std::mutex mutex_;
    };

} // namespace localization_dev

#endif // LOCALIZATION_DEV__ODOM_TF_HPP_