#include "local_path/noisy_odom_node.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

using Point = geometry_msgs::msg::Point;
using Odometry = nav_msgs::msg::Odometry;

namespace f7_sim {

NoisyOdomNode::NoisyOdomNode(const rclcpp::NodeOptions &options): rclcpp::Node("noisy_odom_node", options) {
  // Define random generator with Gaussian distribution
  this->xy_dist = std::normal_distribution<double>(xy_mean, xy_stddev);
  this->th_dist = std::normal_distribution<double>(th_mean, th_stddev);

  auto real_odom_callback = [this](const Odometry &msg) -> void {

    Point noisy_odom;
    double roll=0.0,pitch=0.0,yaw=0.0;
    tf2::getEulerYPR(msg.pose.pose.orientation,yaw,pitch,roll); // quaternion to euler

    noisy_odom.x = msg.pose.pose.position.x;
    noisy_odom.y = msg.pose.pose.position.y;
    noisy_odom.z = yaw;

    if(!DISABLE_NOISE){ //hppでdefineされている
    noisy_odom.x += this->xy_dist(this->generator);
    noisy_odom.y += this->xy_dist(this->generator);
    noisy_odom.z += this->th_dist(this->generator);
    }

    this->noisy_odom_pub->publish(noisy_odom);
  };

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  this->real_odom_sub = this->create_subscription<Odometry>("odom", qos, real_odom_callback);
  this->noisy_odom_pub =this->create_publisher<Point>("noisy_odom", qos);
}

NoisyOdomNode::~NoisyOdomNode() {}

} // namespace localization_dev

RCLCPP_COMPONENTS_REGISTER_NODE(f7_sim::NoisyOdomNode)

