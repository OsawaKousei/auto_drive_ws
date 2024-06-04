

// ********************************************************************************************************************
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using Odometry = nav_msgs::msg::Odometry;

// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
Point point_init;
point_init.x = 0.0;
point_init.y = 0.0;
point_init.z = 0.0;
// ********************************************************************************************************************
// クラスの定義
// ********************************************************************************************************************
class PurePNode : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<Point>::SharedPtr fixed_pos_pub;
  rclcpp::Subscription<TFMessage>::SharedPtr odom_sub;
  // rclcpp::Subscription<Point>::SharedPtr pos_sub;
  Point fixed_pos_msg;
  Point odom_pos_data = point_init;
  Point prior_pos_data = point_init;

public:
  PurePNode() : Node("pure_p_node") {
    RCLCPP_INFO(this->get_logger(), "pure_p_node is activated");
    // パラメータの宣言と取得

    auto timer_callback = [this]() -> void {
      // TODO:カルマンフィルタの実装
      //今はただオドメトリをそのまま送っている
      fix_pos_msg.x = odom_pos_data.x;
      fix_pos_msg.y = odom_pos_data.y;
      fix_pos_msg.z = odom_pos_data.z;
      fixed_pos_pub->publish(fixed_pos_msg);
    };

    auto odom_callback = [this](const Odometry &msg) -> void {
      // オドメトリからの位置を取得
      // 現在位置を取得
      tf2::Quaternion quat;
      quat.setW(msg.pose.pose.orientation.w);
      quat.setX(msg.pose.pose.orientation.x);
      quat.setY(msg.pose.pose.orientation.y);
      quat.setZ(msg.pose.pose.orientation.z);

      pos_data.x = msg.pose.pose.position.x;
      pos_data.y = msg.pose.pose.position.y;
      pos_data.z =
          atan2(2.0 * (quat.x() * quat.y() + quat.z() * quat.w()),
                1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    };

    auto prior_pos_callback = [this](const Point &msg) -> void {
      // localizationされた位置の取得
      prior_pos_data.x = msg.x;
      prior_pos_data.y = msg.y;
      prior_pos_data.z = msg.z;
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    fixed_pos_pub = this->create_publisher<Point>("fixed_pos", qos);
    prior_pos_sub =
        this->create_subscription<Point>("prior_pos", qos, prior_pos_callback);
    odom_sub = this->create_subscription<Odometry>("odom", qos, odom_callback);
    timer = this->create_wall_timer(10ms, timer_callback);
  }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}