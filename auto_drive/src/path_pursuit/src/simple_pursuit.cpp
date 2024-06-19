#include "path_pursuit/simple_pursuit.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "pid.hpp"

#include <fstream>

using namespace std::chrono_literals;

namespace path_pursuit {
SimplePursuit::SimplePursuit(const rclcpp::NodeOptions &options)
    : rclcpp::Node("simple_pursuit", options) {

  // configure parameters
  declare_parameter("kp", 0.0);
  declare_parameter("ki", 0.0);
  declare_parameter("kd", 0.0);
  declare_parameter("ctrl_priod", 0.1);
  declare_parameter("path_dir", "./path.csv");
  declare_parameter("distance_threshold", 0.1);
  declare_parameter("threshold_modify_rate", 1.0);
  get_parameter("kp", kp);
  get_parameter("ki", ki);
  get_parameter("kd", kd);
  get_parameter("ctrl_priod", ctrl_priod_);
  get_parameter("path_dir", path_dir_);
  get_parameter("distance_threshold", distance_threshold_);
  get_parameter("threshold_modify_rate", threshold_modify_rate_);
  std::cout << "kp: " << kp << " ki: " << ki << " kd: " << kd << std::endl;
  std::cout << "ctrl_priod: " << ctrl_priod_ << std::endl;
  std::cout << "path_dir: " << path_dir_ << std::endl;
  std::cout << "distance_threshold: " << distance_threshold_ << std::endl;
  std::cout << "threshold_modify_rate: " << threshold_modify_rate_ << std::endl;

  pid_x = PID(kp, ki, kd);
  pid_x.set_dt(ctrl_priod_);
  pid_y = PID(kp, ki, kd);
  pid_y.set_dt(ctrl_priod_);

  // read path from csv file
  std::ifstream ifs(path_dir_);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", path_dir_.c_str());
    return;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    std::string token;
    std::vector<double> point;
    while (std::getline(iss, token, ',')) {
      point.push_back(std::stod(token));
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z = 0.0;
    path_.poses.push_back(pose);
  }
  path_.header.frame_id = "map";

  std::cout << "read path from csv file: " << path_dir_ << std::endl;
  std::cout << "path size: " << path_.poses.size() << std::endl;

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });
  
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  local_path_pub_ = create_publisher<nav_msgs::msg::Path>("local_path", 1);
}

SimplePursuit::~SimplePursuit() {}

// TODO : syncronize path and odom

void SimplePursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // get current position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // check if path is empty
  if (path_.poses.size() == 0) {
    return;
  }

  double dx = path_.poses[0].pose.position.x - msg->pose.pose.position.x;
  double dy = path_.poses[0].pose.position.y - msg->pose.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // std::cout << "distance: " << distance << std::endl;

  if (distance < distance_threshold_){
    // update path
    path_.poses.erase(path_.poses.begin());

    double dx = path_.poses[1].pose.position.x - path_.poses[0].pose.position.x;
    double dy = path_.poses[1].pose.position.y - path_.poses[0].pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    modified_threshold_ = distance_threshold_ + threshold_modify_rate_ * distance *distance;
    std::cout << "modified_threshold: " << modified_threshold_ << std::endl;

    // TODO : ゴールに到達した際の適切な処理を追加
  }

  // publish local path
  local_path_pub_->publish(path_);

  // pid control
  double target_x = path_.poses[2].pose.position.x;
  double target_y = path_.poses[2].pose.position.y;
  double cmd_x = pid_x.pid_ctrl(x, target_x);
  double cmd_y = pid_y.pid_ctrl(y, target_y);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = cmd_x;
  cmd_vel.linear.y = cmd_y;

  this->cmd_vel_pub_->publish(cmd_vel);
}

} // namespace path_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(path_pursuit::SimplePursuit)
