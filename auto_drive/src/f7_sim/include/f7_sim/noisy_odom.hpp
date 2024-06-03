#ifndef F7_SIM__NOISY_ODOM_HPP_
#define F7_SIM__NOISY_ODOM_HPP_

#include "f7_sim/visibility_control.h"

#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h> //getEulerYPR

#define DISABLE_NOISE 1

namespace f7_sim {

class NoisyOdom : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit NoisyOdom(const rclcpp::NodeOptions &options);
  virtual ~NoisyOdom();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr noisy_odom_pub;

  //! mean of the Gaussian distribution for x and y
  double xy_mean;
  //! standard deviation of the Gaussian distribution for x and y
  double xy_stddev;
  //! mean of the Gaussian distribution for theta
  double th_mean;
  //! standard deviation of the Gaussian distribution for theta
  double th_stddev;

  //! random number generator
  std::default_random_engine generator;
  std::normal_distribution<double> xy_dist;
  std::normal_distribution<double> th_dist;

  std::mutex mutex_;
};

} // namespace f7_sim

#endif // F7_SIM__NOISY_ODOM_HPP_
