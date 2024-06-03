// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Bool.hpp"
#include "geometry_msgs/msg/Twist.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using Bool = std_msgs::msg::Bool;
using Point = geometry_msgs::msg::Point;
using Twist = geometry_msgs::msg::Twist;

// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
Point point_init;point_init.x = 0.0;point_init.y = 0.0;point_init.z = 0.0;

const float EPSILON_RAD = M_PI/180.0; // 許容誤差[rad]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class PurePNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr cmd_pos_pub;
    rclcpp::Subscription<TFMessage>::SharedPtr odom_sub;
    rclcpp::Subscription<Point>::SharedPtr cmd_pos_sub;
    Point cmd_pos_msg;
    Point pos_data = point_init; 

    public:
    PurePNode() : Node("pure_p_node"){
        RCLCPP_INFO(this->get_logger(), "pure_p_node is activated");
        // パラメータの宣言と取得

        auto timer_callback = [this]() -> void{
            cmd_pos_pub->publish(cmd_pos_msg);
        };

        // auto odom_callback = [this](const TFMessage& msg) -> void{
        //     // 位置指令を取得
        //     pos_data.x = msg.transform.translation.x;
        //     pos_data.y = msg.transform.translation.y;
        //     pos_data.z = atan2(2.0 * (msg.transform.rotation.x * msg.transform.rotation.y + msg.transform.rotation.z * msg.transform.rotation.w),
        //                 1.0 - 2.0 * (msg.transform.rotation.y * msg.transform.rotation.y + msg.transform.rotation.z * msg.transform.rotation.z));
        // };

        auto fixed_pos_callback = [this](const Point& msg) -> void{
            // 修正された現在位置を取得
            pos_data.x = msg.x;
            pos_data.y = msg.y;
            pos_data.z = msg.z;
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        cmd_pos_pub = this->create_publisher<Point>("cmd_pos", qos);
        //odom_sub = this->create_subscription<TFMessage>("/odom/tf", qos, odom_callback);
        fixed_pos_sub = this->create_subscription<Point>("fixed_pos", qos, fixed_pos_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }

};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PurePNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}