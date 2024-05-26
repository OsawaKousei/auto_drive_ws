#ifndef LOCALIZATION_DEV__LOCALIZATION_TEST_NODE_HPP_
#define LOCALIZATION_DEV__LOCALIZATION_TEST_NODE_HPP_

#include "localization_dev/visibility_control.h"
#include <rclcpp/rclcpp.hpp>


namespace localization_dev
{

class LocalizationTest : public rclcpp::Node
{
public:
  TUTORIAL_PUBLIC
  explicit LocalizationTest(const rclcpp::NodeOptions & options);

  virtual ~LocalizationTest();

private:
   
};

}  // namespace localization_dev

#endif  // LOCALIZATION_DEV__LOCALIZATION_TEST_NODE_HPP_