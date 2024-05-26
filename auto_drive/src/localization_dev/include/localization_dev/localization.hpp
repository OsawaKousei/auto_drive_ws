#ifndef LOCALIZATION_DEV__LOCALIZATION_HPP_
#define LOCALIZATION_DEV__LOCALIZATION_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>


namespace localization_dev
{

class Localization : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit Localization(const rclcpp::NodeOptions & options);

  virtual ~Localization();

private:
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__LOCALIZATION_HPP_