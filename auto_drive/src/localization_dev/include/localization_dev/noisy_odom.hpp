#ifndef LOCALIZATION_DEV__NOISY_ODOM_HPP_
#define LOCALIZATION_DEV__NOISY_ODOM_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>


namespace localization_dev
{

class NoisyOdom : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit NoisyOdom(const rclcpp::NodeOptions & options);

  virtual ~NoisyOdom();

private:

};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__NOISY_ODOM_HPP_