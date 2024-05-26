#ifndef LOCALIZATION_DEV__NOISY_ODOM_HPP_
#define LOCALIZATION_DEV__NOISY_ODOM_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>


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
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__NOISY_ODOM_HPP_