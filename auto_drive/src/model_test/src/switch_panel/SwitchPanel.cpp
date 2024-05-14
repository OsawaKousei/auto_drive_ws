#include "SwitchPanel.hpp"

#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

namespace model_test
{

SwitchPanel::SwitchPanel() : Plugin()
{
  CreateIgnitionIf();
}

SwitchPanel::~SwitchPanel()
{
}

void SwitchPanel::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (!_pluginElem) {
    return;
  }

  // 意図せずforward_speedとreverse_speedが上書きされるのでコメントアウト
  // auto forward_speed_elem = _pluginElem->FirstChildElement("forward_speed");
  // if (nullptr != forward_speed_elem)
  // {
  //   forward_speed_ = forward_speed_elem->FloatText();
  // }

  // auto reverse_speed_elem = _pluginElem->FirstChildElement("reverse_speed");
  // if (nullptr != reverse_speed_elem)
  // {
  //   reverse_speed_ = reverse_speed_elem->FloatText();
  // }
}

void SwitchPanel::OnForwardButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(forward_speed_);
  wheel_speed_pub_.Publish(float_msg);
}

void SwitchPanel::OnStopButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(0.0f);
  wheel_speed_pub_.Publish(float_msg);
}

void SwitchPanel::OnBackwardButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(reverse_speed_);
  wheel_speed_pub_.Publish(float_msg);
}

void SwitchPanel::OnRightButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(forward_speed_);
  base_speed_pub_.Publish(float_msg);
}

void SwitchPanel::OnLeftButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(reverse_speed_);
  base_speed_pub_.Publish(float_msg);
}

void SwitchPanel::CreateIgnitionIf(void){
  this->wheel_speed_pub_ = this->node_.Advertise<ignition::msgs::Float>("wheel_speed");
  this->base_speed_pub_ = this->node_.Advertise<ignition::msgs::Float>("base_speed");
}

}

// Register this plugin
IGNITION_ADD_PLUGIN(model_test::SwitchPanel, ignition::gui::Plugin)
