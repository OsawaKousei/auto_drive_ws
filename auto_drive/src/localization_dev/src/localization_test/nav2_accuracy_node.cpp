#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class Nav2AccuracyNode : public rclcpp::Node {
public:
  Nav2AccuracyNode() : Node("nav2_accuracy_node") {
    estimated_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 1,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          mutex_.lock();
          double accuracy = 1 / (1 + cbrt(pow(msg->pose.pose.position.x - real_pose.pose.pose.position.x, 2) +
                                           pow(msg->pose.pose.position.y - real_pose.pose.pose.position.y, 2) +
                                           pow(get_yaw(msg->pose.pose.orientation) - get_yaw(real_pose.pose.pose.orientation), 2)));

          auto accuracy_msg = std_msgs::msg::Float64();
          accuracy_msg.data = accuracy;
          accuracy_pub->publish(accuracy_msg);
          RCLCPP_INFO(this->get_logger(), "Real pose: x: %f, y: %f, th: %f", real_pose.pose.pose.position.x,
                      real_pose.pose.pose.position.y, get_yaw(real_pose.pose.pose.orientation));
          RCLCPP_INFO(this->get_logger(), "Estimated pose: x: %f, y: %f, th: %f", msg->pose.pose.position.x,
                      msg->pose.pose.position.y, get_yaw(msg->pose.pose.orientation));
          RCLCPP_INFO(this->get_logger(), "Estimate Accuracy: %f", accuracy);
          mutex_.unlock();
        });

    real_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          mutex_.lock();
          real_pose = *msg;
          mutex_.unlock();
        });

    accuracy_pub = this->create_publisher<std_msgs::msg::Float64>("accuracy", 1);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_pose_sub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accuracy_pub;

  nav_msgs::msg::Odometry real_pose;
  float accuracy;
  std::mutex mutex_;

  float get_yaw(geometry_msgs::msg::Quaternion q) {
    tf2::Quaternion tf2_q;
    tf2::fromMsg(q, tf2_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2AccuracyNode>());
  rclcpp::shutdown();
  return 0;
}