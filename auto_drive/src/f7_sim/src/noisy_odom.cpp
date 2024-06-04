#include "f7_sim/noisy_odom.hpp"
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

using Point = geometry_msgs::msg::Point;
using Odometry = nav_msgs::msg::Odometry;

float noise_x = 0;
float noise_y = 0;
float noise_z = 0;

namespace f7_sim {

NoisyOdom::NoisyOdom(const rclcpp::NodeOptions &options)
    : rclcpp::Node("noisy_odom", options) {
    
  declare_parameter("enable_noise", 1);
  declare_parameter("xy_mean", 1.0);
  declare_parameter("xy_stddev", 1.0);
  declare_parameter("th_mean", 1.0);
  declare_parameter("th_stddev", 1.0);
  get_parameter("enable_noise", enable_noise);
  get_parameter("xy_mean", xy_mean);
  get_parameter("xy_stddev", xy_stddev);
  get_parameter("th_mean", xy_mean);
  get_parameter("th_stddev", th_stddev);
  std::cout << "xy_mean: " << xy_mean << std::endl;
  std::cout << "xy_stddev: " << xy_stddev << std::endl;
  std::cout << "th_mean: " << th_mean << std::endl;
  std::cout << "th_stddev: " << th_stddev << std::endl;
  
  // Define random generator with Gaussian distribution
  this->xy_dist = std::normal_distribution<double>(xy_mean, xy_stddev);
  this->th_dist = std::normal_distribution<double>(th_mean, th_stddev);

  auto real_odom_callback = [this](const Odometry &msg) -> void {
    mutex_.lock();
    Point noisy_odom;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::getEulerYPR(msg.pose.pose.orientation, yaw, pitch,
                     roll); // quaternion to euler

    noisy_odom.x = msg.pose.pose.position.x;
    noisy_odom.y = msg.pose.pose.position.y;
    noisy_odom.z = yaw;

    if (enable_noise) {
      noise_x += this->xy_dist(this->generator);
      noise_y += this->xy_dist(this->generator);
      noise_z += this->th_dist(this->generator);

      noisy_odom.x = msg.pose.pose.position.x + noise_x;
      noisy_odom.y = msg.pose.pose.position.y + noise_y;
      noisy_odom.z = yaw + noise_z;
    }

    this->noisy_odom_pub->publish(noisy_odom);
    mutex_.unlock();
  };

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  this->real_odom_sub =
      this->create_subscription<Odometry>("odom", qos, real_odom_callback);
  this->noisy_odom_pub = this->create_publisher<Point>("noisy_odom", qos);
}

NoisyOdom::~NoisyOdom() {}

} // namespace f7_sim

RCLCPP_COMPONENTS_REGISTER_NODE(f7_sim::NoisyOdom)