#include "localization_dev/pc_mapping/odom_tf.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace localization_dev {

OdomTf::OdomTf(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odom_tf", options) {
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

// デストラクタ
OdomTf::~OdomTf() {}

void OdomTf::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  mutex_.lock();
  // publish tf
  odom_tf.header.stamp = msg->header.stamp;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = msg->pose.pose.position.x;
  odom_tf.transform.translation.y = msg->pose.pose.position.y;
  odom_tf.transform.translation.z = msg->pose.pose.position.z;
  odom_tf.transform.rotation = msg->pose.pose.orientation;
  tf_broadcaster->sendTransform(odom_tf);
  mutex_.unlock();
}
} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(localization_dev::OdomTf)