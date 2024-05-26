// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/utils.h>

using namespace std::chrono_literals;
using Bool = std_msgs::msg::Bool;
using Point = geometry_msgs::msg::Point;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

struct PID_PARAM{
    float P;
    float I;
    float D;
};

struct Point2D{
    Point2D(float X,float Y,float Z):
        x{X}, y{Y}, z{Z} {}
    float x;
    float y;
    float z; //このノードではzをyawとして扱う
};

//TODO: operatorでPoint2Dの演算を定義してコードを見やすくする

// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const struct Point2D point_init = {0.0,0.0,0.0};

const struct PID_PARAM pos_linear_pid={0.1,0.01,0.001};
const struct PID_PARAM pos_angular_pid={0.1,0.01,0.001};

const double MAX_VEL_M = 0.5; // 最大速度[m/s]
const double MAX_VEL_RAD = 0.5; // 最大角速度[rad/s]
const float EPSILON_M = 0.03; // 許容誤差[m]
const float EPSILON_RAD = M_PI/180.0; // 許容誤差[rad]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class F7_SIMNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Point>::SharedPtr cmd_pos_sub;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<Bool>::SharedPtr finish_sub;
    Twist cmd_vel_msg;
    struct Point2D pos_data = point_init; 
    struct Point2D pos_cmd_data = point_init; // サブスクライブしたそのままの値
    struct Point2D pos_error_I = point_init;
    struct Point2D prior_pos_error = point_init; // 前回の誤差
    bool flag_f = false;

    public:
    F7_SIMNode() : Node("f7_sim_node"){
        RCLCPP_INFO(this->get_logger(), "f7_sim_node is activated");
        // パラメータの宣言と取得

        auto timer_callback = [this]() -> void{
            struct Point2D pos_error = point_init; //目標位置との差分
            pos_error.x = pos_cmd_data.x - pos_data.x;
            pos_error.y = pos_cmd_data.y - pos_data.y;
            pos_error.z = pos_cmd_data.z - pos_data.z;

            if((sqrt(pos_error.x*pos_error.x + pos_error.y*pos_error.y) > EPSILON_M or pos_error.z > EPSILON_RAD)
                and !flag_f){

                //差分を積分
                pos_error_I.x += (prior_pos_error.x + pos_error.x)/2.0;
                pos_error_I.y += (prior_pos_error.y + pos_error.y)/2.0;
                pos_error_I.z += (prior_pos_error.z + pos_error.z)/2.0;

                //PID制御
                cmd_vel_msg.linear.x = std::max(-MAX_VEL_M,std::min((double)(pos_error.x*pos_linear_pid.P + pos_error_I.x*pos_linear_pid.I + (prior_pos_error.x - pos_error.x)*pos_linear_pid.D), MAX_VEL_M)); // 最大でMAX_VEL_M[m/s]
                cmd_vel_msg.linear.y = std::max(-MAX_VEL_M,std::min((double)(pos_error.y*pos_linear_pid.P + pos_error_I.y*pos_linear_pid.I + (prior_pos_error.y - pos_error.y)*pos_linear_pid.D), MAX_VEL_M)); // 最大でMAX_VEL_M[m/s]
                cmd_vel_msg.angular.z = std::max(-MAX_VEL_RAD,std::min((double)(pos_error.z*pos_angular_pid.P + pos_error_I.z*pos_angular_pid.I + (prior_pos_error.z - pos_error.z)*pos_angular_pid.D), MAX_VEL_M)); // 最大でMAX_VEL_RAD[rad/s]
                //TODO:不完全微分の導入

                //差分を保存
                prior_pos_error = pos_error;
            }else{
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.linear.y = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                
                pos_data = point_init; 
                pos_cmd_data = point_init;
                pos_error_I = point_init;
                prior_pos_error = point_init;
            }

            cmd_vel_pub->publish(cmd_vel_msg);
        };

        auto cmd_pos_callback = [this](const Point& msg) -> void{
            // 位置指令を取得
            pos_cmd_data.x = msg.x;
            pos_cmd_data.y = msg.y;
            pos_cmd_data.z = msg.z;
        };

        auto odom_callback = [this](const Odometry& msg) -> void{
            // 現在位置を取得
            tf2::Quaternion quat;
            quat.setW(msg.pose.pose.orientation.w);
            quat.setX(msg.pose.pose.orientation.x);
            quat.setY(msg.pose.pose.orientation.y);
            quat.setZ(msg.pose.pose.orientation.z);

            pos_data.x = msg.pose.pose.position.x;
            pos_data.y = msg.pose.pose.position.y;
            pos_data.z = atan2(2.0 * (quat.x() * quat.y() + quat.z() * quat.w()),
                        1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
        };

        auto flag_finish_callback = [this](const Bool& msg) -> void{
            flag_f = msg.data;
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", qos);
        cmd_pos_sub = this->create_subscription<Point>("cmd_pos", qos, cmd_pos_callback);
        odom_sub = this->create_subscription<Odometry>("odom", qos, odom_callback);
        finish_sub = this->create_subscription<Bool>("flag_finish", qos, flag_finish_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }

};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<F7_SIMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}