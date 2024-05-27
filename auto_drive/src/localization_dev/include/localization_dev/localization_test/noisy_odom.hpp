#ifndef LOCALIZATION_DEV__NOISY_ODOM_HPP_
#define LOCALIZATION_DEV__NOISY_ODOM_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>


namespace localization_dev
{

class NoisyOdom : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit NoisyOdom(const rclcpp::NodeOptions & options);

  virtual ~NoisyOdom();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_pub;

    // Define random generator with Gaussian distribution
    const double xy_mean = 0.0;
    const double xy_stddev = 0.001;
    const double th_mean = 0.0;
    const double th_stddev = 0.1;
    std::default_random_engine generator;
    std::normal_distribution<double> xy_dist;
    std::normal_distribution<double> th_dist;
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__NOISY_ODOM_HPP_