#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/utils.h>

using namespace std::chrono_literals;

class IgnDebugNode : public rclcpp::Node {
public:

    IgnDebugNode() : Node("pubsub_node") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("practice_topic", 10);

        auto publish_msg_callback = [this]() -> void {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = vx_;
            message.linear.y = vy_;
            message.angular.z = vth_;
            this->publisher_->publish(message);
        }; 

        timer_ = this->create_wall_timer(500ms, publish_msg_callback);
  
        auto topic_callback = [this](const nav_msgs::msg::Odometry &msg) -> void {

            //set the previous position, orientation and time
            prev_x_ = x_;
            prev_y_ = y_;
            prev_th_ = th_;
            prev_time_ = time_;
            
            //get the position, orientation and time
            x_ = msg.pose.pose.position.x;
            y_ = msg.pose.pose.position.y;
            tf2::Quaternion q(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w);
            tf2::getEulerYPR(q, yaw, pitch, roll);
            th_ = yaw;
            time_ = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

            //calculate the difference in position, orientation and time
            float dt = time_ - prev_time_;
            float dx = x_ - prev_x_;
            float dy = y_ - prev_y_;
            float dth = th_ - prev_th_;

            //rotate the difference in position by the orientation of the robot
            tf2::Matrix3x3 rotation_matrix;
            rotation_matrix.setRPY(0.0, 0.0, th_);
            tf2::Vector3 v(dx, dy, 0.0);
            v = rotation_matrix * v;
            dx = v.getX();
            dy = v.getY();

            //calculate the velocity
            vx_ = dx / dt;
            vy_ = dy / dt;
            vth_ = dth / dt;

            //show the velocity if the robot is moving
            if (vx_ != 0 || vy_ != 0 || vth_ != 0){
                RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, vth: %f", vx_, vy_, vth_);
            }
        }; 

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    float x_ = 0.0;
    float prev_x_ = 0.0;
    float y_ = 0.0;
    float prev_y_ = 0.0;
    float th_ = 0.0;
    float prev_th_ = 0.0;
    double roll, pitch, yaw;
    float time_ = 0.0;
    float prev_time_ = 0.0;
    float vx_ = 0.0;
    float vy_ = 0.0;
    float vth_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IgnDebugNode>());
    rclcpp::shutdown();
    return 0;
}