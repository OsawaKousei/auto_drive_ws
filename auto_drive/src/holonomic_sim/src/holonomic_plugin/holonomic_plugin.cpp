
#include "holonomic_plugin.hpp"
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/msgs/twist.pb.h>

namespace holonomic_sim
{

HolonomicPlugin::HolonomicPlugin()
{
  CreateIgnitionIf();
}

HolonomicPlugin::~HolonomicPlugin()
{
}

void HolonomicPlugin::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  (void)_ecm;
  (void)_eventMgr;
  model_ = ignition::gazebo::Model(_entity);

  auto wheel_ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr wheel_joint_elem = wheel_ptr->GetElement("wheel_joint");
  if (wheel_joint_elem) {
    wheel_joint_name_ = wheel_joint_elem->Get<std::string>();
  } else {
    ignerr << "sdf wheel_joint not found" << std::endl;
  }

  auto base_ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr base_joint_elem = base_ptr->GetElement("base_joint");
  if (base_joint_elem) {
    base_joint_name_ = base_joint_elem->Get<std::string>();
  } else {
    ignerr << "sdf base_joint not found" << std::endl;
  }
}

void HolonomicPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    (void)_info;
    (void)_ecm;
    ignition::gazebo::Entity wheel_joint = model_.JointByName(_ecm, wheel_joint_name_);
    if (wheel_joint == ignition::gazebo::kNullEntity){
      ignerr << wheel_joint_name_ <<" not found" << std::endl;
      return;
    }

    ignition::gazebo::Entity base_joint = model_.JointByName(_ecm, base_joint_name_);
    if (base_joint == ignition::gazebo::kNullEntity){
      ignerr << base_joint_name_ <<" not found" << std::endl;
      return;
    }

    auto wheel_rot_vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(wheel_joint);
    if (wheel_rot_vel != nullptr)
    {
      *wheel_rot_vel = ignition::gazebo::components::JointVelocityCmd({wheel_rot_vel_});
    }
    else {
      _ecm.CreateComponent(wheel_joint, ignition::gazebo::components::JointVelocityCmd({wheel_rot_vel_}));
    }

    auto base_pos = _ecm.Component<ignition::gazebo::components::JointPositionReset>(base_joint);
    if (base_pos != nullptr)
    {
      *base_pos = ignition::gazebo::components::JointPositionReset({base_pos_});
    }
    else {
      _ecm.CreateComponent(base_joint, ignition::gazebo::components::JointPositionReset({base_pos_}));
    }

    auto base_vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(base_joint);
    if (base_vel != nullptr)
    {
      *base_vel = ignition::gazebo::components::JointVelocityCmd({base_vel_});
    }
    else {
      _ecm.CreateComponent(base_joint, ignition::gazebo::components::JointVelocityCmd({base_vel_}));
    }
 }

void HolonomicPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

void HolonomicPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

void HolonomicPlugin::CreateIgnitionIf(void){
  this->node_.Subscribe("cmd_vel", &HolonomicPlugin::OnCmdVelMessage, this);
}

void HolonomicPlugin::OnCmdVelMessage(const ignition::msgs::Twist & msg)
{
  float x = msg.linear().x();
  float y = msg.linear().y();
  wheel_vel_ = sqrt(x*x + y*y);
  wheel_rot_vel_ = wheel_vel_/wheel_radius_;
  base_pos_ = atan2(y, x);
  base_vel_ = base_rot_param_*msg.angular().z();
}

}

#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(
    holonomic_sim::HolonomicPlugin,
    ignition::gazebo::System,
    holonomic_sim::HolonomicPlugin::ISystemConfigure,
    holonomic_sim::HolonomicPlugin::ISystemPreUpdate,
    holonomic_sim::HolonomicPlugin::ISystemUpdate,
    holonomic_sim::HolonomicPlugin::ISystemPostUpdate)
