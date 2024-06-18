#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error "undefined reference to 'tf2::fromMsg(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'"
#include <tf2/utils.h> //getEulerYPR

#include "path_pursuit/pid.hpp"


using namespace std::chrono_literals;

class PIDTestNode : public rclcpp::Node {
public:
  PIDTestNode() : Node("pid_test_node") {

    // configure parameters
    declare_parameter("kp", 0.0);
    declare_parameter("ki", 0.0);
    declare_parameter("kd", 0.0);
    get_parameter("kp", kp);
    get_parameter("ki", ki);
    get_parameter("kd", kd);
    std::cout << "kp: " << kp << " ki: " << ki << " kd: " << kd << std::endl;

    // create a PID controller
    pid_x = PID(kp, ki, kd);
    pid_y = PID(kp, ki, kd);

    // create a subscriber to the pose topic
    pose_sub = create_subscription<geometry_msgs::msg::Pose>(
      "pose", 1, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        mutex_.lock();
        pose_ = *msg;
        mutex_.unlock();
      });

    // create a subscriber to the odometry topic
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg);});

    // create a publisher to the twist topic
    twist_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Pose pose_;
  double kp, ki, kd;

  PID pid_x = PID(0.0, 0.0, 0.0);
  PID pid_y = PID(0.0, 0.0, 0.0);

  std::mutex mutex_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    mutex_.lock();

    odom_ = *msg;
    double yaw, pitch, roll;
    tf2::getEulerYPR(this->odom_.pose.pose.orientation, yaw, pitch,roll); // quaternion to euler

    std::cout << "now_x: " << odom_.pose.pose.position.x << "now_y: " << odom_.pose.pose.position.y << std::endl;
    std::cout << "target_x: " << pose_.position.x << " target_y: " << pose_.position.y << std::endl;

    double cmd_x;
    double cmd_y;
    
    cmd_x = pid_x.pid_ctrl(odom_.pose.pose.position.x, pose_.position.x);
    cmd_y = pid_y.pid_ctrl(odom_.pose.pose.position.y, pose_.position.y);

    std::cout << "cmd_x: " << cmd_x << " cmd_y: " << cmd_y << std::endl;

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = cmd_x;
    twist_msg.linear.y = cmd_y;
    twist_msg.angular.z = 0.0;

    // publish the twist message
    twist_pub->publish(twist_msg);
    mutex_.unlock();
  }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDTestNode>());
    rclcpp::shutdown();
    return 0;
}