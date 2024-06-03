#ifndef F7_SIM__NOISY_ODOM_NODE_HPP_
#define F7_SIM__NOISY_ODOM_NODE_HPP_

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error 
#include <tf2/utils.h> //getEulerYPR

#define DISABLE_NOISE 1

namespace f7_sim {

class NoisyOdomNode : public rclcpp::Node {
public:
  explicit NoisyOdomNode(const rclcpp::NodeOptions &options);
  virtual ~NoisyOdomNode();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr noisy_odom_pub;

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

};

} // namespace f7_sim

#endif // F7_SIM__NOISY_ODOM_NODE_HPP_

