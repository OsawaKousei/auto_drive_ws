#include "HolonomicPanel.hpp"

#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

namespace model_test
{

HolonomicPanel::HolonomicPanel() : Plugin()
{
  CreateIgnitionIf();
}

HolonomicPanel::~HolonomicPanel()
{
}

void HolonomicPanel::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (!_pluginElem) {
    return;
  }

  // 意図せずforward_speedとreverse_speedが上書きされるのでコメントアウト
  // auto wheel_speed_elem = _pluginElem->FirstChildElement("forward_speed");
  // if (nullptr != wheel_speed_elem)
  // {
  //   wheel_speed_ = wheel_speed_elem->FloatText();
  // }

  // auto wheel_speed_elem = _pluginElem->FirstChildElement("reverse_speed");
  // if (nullptr != wheel_speed_elem)
  // {
  //   wheel_speed_ = wheel_speed_elem->FloatText();
  // }
}

void HolonomicPanel::OnForwardButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(0.0f);
  base_pos_pub_.Publish(float_msg);
}

void HolonomicPanel::OnBackwardButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(3.14f);
  base_pos_pub_.Publish(float_msg);
}

void HolonomicPanel::OnRightButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(1.57f);
  base_pos_pub_.Publish(float_msg);
}

void HolonomicPanel::OnLeftButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(-1.57f);
  base_pos_pub_.Publish(float_msg);
}

void HolonomicPanel::OnGoButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(wheel_speed_);
  wheel_speed_pub_.Publish(float_msg);
}

void HolonomicPanel::OnStopButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(0.0f);
  wheel_speed_pub_.Publish(float_msg);
}

void HolonomicPanel::CreateIgnitionIf(void){
  this->wheel_speed_pub_ = this->node_.Advertise<ignition::msgs::Float>("wheel_speed");
  this->base_pos_pub_ = this->node_.Advertise<ignition::msgs::Float>("base_pos");
}

}

// Register this plugin
IGNITION_ADD_PLUGIN(model_test::HolonomicPanel, ignition::gui::Plugin)
