#include "f7_sim/f7_sim.hpp"
#include "f7_sim/util_functions.hpp"
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
using Bool = std_msgs::msg::Bool;
using Point = geometry_msgs::msg::Point;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

namespace f7_sim {

F7Sim::F7Sim(const rclcpp::NodeOptions &options)
    : rclcpp::Node("f7_sim", options) {
  RCLCPP_INFO(this->get_logger(), "f7_sim_node is activated");

  declare_parameter("abs_coordinate", 1);
  get_parameter("abs_coordinate", abs_coordinate);
  // configure parameters
  std::cout << "abs_coordinate: " << abs_coordinate << std::endl;

  auto timer_callback = [this]() -> void {
    mutex_.lock();
    // 絶対座標用
    if (abs_coordinate) {
      pos_error.x = pos_cmd_data.x - pos_data.x;
      pos_error.y = pos_cmd_data.y - pos_data.y;
      pos_error.z = pos_cmd_data.z - pos_data.z;
    }

    // グローバル座標と向きが同じ相対座標

    if ((sqrt(pos_error.x * pos_error.x + pos_error.y * pos_error.y) >
             EPSILON_M or
         pos_error.z > EPSILON_RAD) and
        !flag_f) {

      // TODO: PIDの終了を距離誤差と角誤差で分離する

      //差分を積分
      pos_error_I.x += (prior_pos_error.x + pos_error.x) / 2.0;
      pos_error_I.y += (prior_pos_error.y + pos_error.y) / 2.0;
      pos_error_I.z += (prior_pos_error.z + pos_error.z) / 2.0;

      // PID制御
      // TODO:不完全微分の導入
      // TODO:アンチワインドアップの修正（今のままではI項がまともに使えない）

      double move_yaw = atan2(pos_error.y, pos_error.x);
      // fix MAX_VEL_M direction （MERGINの分だけ少し変になる
      double X_M_V_M = abs((MAX_VEL_M - MERGIN) * cos(move_yaw)) + MERGIN;
      double Y_M_V_M = abs((MAX_VEL_M - MERGIN) * sin(move_yaw)) + MERGIN;
      // double Y_M_V_M = MAX_VEL_M;

      double global_x; // 最大でMAX_VEL_M[m/s]
      double global_y; // 最大でMAX_VEL_M[m/s]
      double global_z; // 最大でMAX_VEL_RAD[rad/s]
      if (chclip(global_x,
                 (double)(pos_error.x * pos_linear_pid.P +
                          pos_error_I.x * pos_linear_pid.I +
                          (prior_pos_error.x - pos_error.x) * pos_linear_pid.D),
                 -X_M_V_M, X_M_V_M)) {
        pos_error_I.x = 0;
        global_x =
            clip((double)(pos_error.x * pos_linear_pid.P +
                          pos_error_I.x * pos_linear_pid.I +
                          (prior_pos_error.x - pos_error.x) * pos_linear_pid.D),
                 -X_M_V_M, X_M_V_M);
      } // 最大でMAX_VEL_M[m/s]
      if (chclip(global_y,
                 (double)(pos_error.y * pos_linear_pid.P +
                          pos_error_I.y * pos_linear_pid.I +
                          (prior_pos_error.y - pos_error.y) * pos_linear_pid.D),
                 -Y_M_V_M, Y_M_V_M)) {
        pos_error_I.y = 0;
        global_y =
            clip((double)(pos_error.y * pos_linear_pid.P +
                          pos_error_I.y * pos_linear_pid.I +
                          (prior_pos_error.y - pos_error.y) * pos_linear_pid.D),
                 -Y_M_V_M, Y_M_V_M);
      } // 最大でMAX_VEL_M[m/s]
      if (chclip(
              global_z,
              (double)(pos_error.z * pos_angular_pid.P +
                       pos_error_I.z * pos_angular_pid.I +
                       (prior_pos_error.z - pos_error.z) * pos_angular_pid.D),
              -MAX_VEL_RAD, MAX_VEL_RAD)) {
        pos_error_I.z = 0;
        global_z = clip(
            (double)(pos_error.z * pos_angular_pid.P +
                     pos_error_I.z * pos_angular_pid.I +
                     (prior_pos_error.z - pos_error.z) * pos_angular_pid.D),
            -MAX_VEL_RAD, MAX_VEL_RAD);
      } // 最大でMAX_VEL_RAD[rad/s]

      cmd_vel_msg.linear.x =
          global_x * cos(pos_data.z) + global_y * sin(pos_data.z);
      cmd_vel_msg.linear.y =
          -global_x * sin(pos_data.z) + global_y * cos(pos_data.z);
      cmd_vel_msg.angular.z = global_z;

      //差分を保存
      prior_pos_error = pos_error;
    } else {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.linear.y = 0.0;
      cmd_vel_msg.angular.z = 0.0;

      pos_error_I = point_init;
      prior_pos_error = point_init;
    }

    cmd_vel_pub->publish(cmd_vel_msg);
    mutex_.unlock();
  };

  auto cmd_pos_callback = [this](const Point &msg) -> void {
    mutex_.lock();
    if (abs_coordinate) {
      // 位置指令を取得
      pos_cmd_data.x = msg.x;
      pos_cmd_data.y = msg.y;
      pos_cmd_data.z = msg.z;
    } else {
      // 相対位置指令を取得
      pos_error.x = msg.x;
      pos_error.y = msg.y;
      pos_error.z = msg.z;
    }
    mutex_.unlock();
  };

  auto odom_callback = [this](const Point &msg) -> void {
    mutex_.lock();
    // 直接odometryをとる場合
    // // 現在位置を取得
    // tf2::Quaternion quat;
    // quat.setW(msg.pose.pose.orientation.w);
    // quat.setX(msg.pose.pose.orientation.x);
    // quat.setY(msg.pose.pose.orientation.y);
    // quat.setZ(msg.pose.pose.orientation.z);

    // pos_data.x = msg.pose.pose.position.x;
    // pos_data.y = msg.pose.pose.position.y;
    // pos_data.z = atan2(2.0 * (quat.x() * quat.y() + quat.z() * quat.w()),
    //             1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));

    // for debug
    // std::cout << pos_data.z << std::endl;

    // noisy_odom_nodeからとる場合
    pos_data.x = msg.x;
    pos_data.y = msg.y;
    pos_data.z = msg.z;
    mutex_.unlock();
  };

  auto flag_finish_callback =
      [this](const Bool &msg) -> void { // TODO:未実装(callback内のif文も要修正)
    mutex_.lock();
    flag_f = msg.data;
    mutex_.unlock();
  };

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  this->cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", qos);
  this->cmd_pos_sub =
      this->create_subscription<Point>("cmd_pos", qos, cmd_pos_callback);
  // this->odom_sub = this->create_subscription<Odometry>("odom", qos,
  // odom_callback);
  this->odom_sub =
      this->create_subscription<Point>("noisy_odom", qos, odom_callback);
  this->finish_sub =
      this->create_subscription<Bool>("flag_finish", qos, flag_finish_callback);
  this->timer = this->create_wall_timer(10ms, timer_callback);
}

F7Sim::~F7Sim() {}

} // namespace f7_sim

RCLCPP_COMPONENTS_REGISTER_NODE(f7_sim::F7Sim)