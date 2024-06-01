
#ifndef F7_SIM__F7_SIM_NODE_HPP_
#define F7_SIM__F7_SIM_NODE_HPP_

// ********************************************************************************************************************
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

#define ABS_COORDINATE 1

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

const struct PID_PARAM pos_linear_pid={1.0,0.00,0.01};
const struct PID_PARAM pos_angular_pid={2.0,0.00,0.01};

const double MAX_VEL_M = 1.0; // 最大速度[m/s]
const double MAX_VEL_RAD = 0.5; // 最大角速度[rad/s]
const float EPSILON_M = 0.03; // 許容誤差[m]
const float EPSILON_RAD = M_PI/120.0; // 許容誤差[rad]
const float MERGIN = 0.1; //最大速度補正時の余裕
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
namespace f7_sim {

class F7_SIMNode : public rclcpp::Node{
    public:
    //TUTORIAL_PUBLIC
    explicit F7_SIMNode(const rclcpp::NodeOptions &options);
    virtual ~F7_SIMNode();
    
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Point>::SharedPtr cmd_pos_sub;
    //rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<Point>::SharedPtr odom_sub;
    rclcpp::Subscription<Bool>::SharedPtr finish_sub;
    Twist cmd_vel_msg;
    struct Point2D pos_data = point_init; 
    struct Point2D pos_cmd_data = point_init; // サブスクライブしたそのままの値
    struct Point2D pos_error = point_init; //目標位置との差分
    struct Point2D pos_error_I = point_init;
    struct Point2D prior_pos_error = point_init; // 前回の誤差
    bool flag_f = false;
};

}


#endif // F7_SIM__F7_SIM_NODE_HPP_