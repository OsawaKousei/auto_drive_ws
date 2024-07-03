/**
 * @file noisy_odom.hpp
 * @brief Header file for the NoisyOdom class
 * @author kousei
 * @date 2024-05-29
 */

#ifndef LOCALIZATION_DEV__NOISY_ODOM_HPP_
#define LOCALIZATION_DEV__NOISY_ODOM_HPP_

#include "localization_dev/visibility_control.h"
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace localization_dev {
/// @brief A class to add noise to the odometry.
/// @details This class adds noise to the odometry and publishes the noisy
/// odometry. The noisy odometry is published as an Odometry message. The real
/// odometry is received from the /odom topic. The noise is added to the x, y,
/// and theta values of the odometry. The noise is generated from a Gaussian
/// distribution
class NoisyOdom : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit NoisyOdom(const rclcpp::NodeOptions &options);
  virtual ~NoisyOdom();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr noisy_odom_pub;

  //! mean of the Gaussian distribution for x and y
  const double xy_mean = 0.0;
  //! standard deviation of the Gaussian distribution for x and y
  const double xy_stddev = 0.001;
  //! mean of the Gaussian distribution for theta
  const double th_mean = 0.0;
  //! standard deviation of the Gaussian distribution for theta
  const double th_stddev = 0.1;

  //! random number generator
  std::default_random_engine generator;
  std::normal_distribution<double> xy_dist;
  std::normal_distribution<double> th_dist;

  std::mutex mutex_;
};

} // namespace localization_dev

#endif // LOCALIZATION_DEV__NOISY_ODOM_HPP_
