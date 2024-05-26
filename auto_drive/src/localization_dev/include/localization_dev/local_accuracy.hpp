#ifndef LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_
#define LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>


namespace localization_dev
{

class LocalAccuracy : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit LocalAccuracy(const rclcpp::NodeOptions & options);

  virtual ~LocalAccuracy();

private:

};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__LOCAL_ACCURACY_HPP_