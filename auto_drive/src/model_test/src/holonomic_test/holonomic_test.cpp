
#include "holonomic_test.hpp"
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/float.pb.h>

namespace model_test {

HolonomicTest::HolonomicTest() { CreateIgnitionIf(); }

HolonomicTest::~HolonomicTest() {}

void HolonomicTest::Configure(const ignition::gazebo::Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              ignition::gazebo::EntityComponentManager &_ecm,
                              ignition::gazebo::EventManager &_eventMgr) {
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

void HolonomicTest::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager &_ecm) {
  (void)_info;
  (void)_ecm;
  ignition::gazebo::Entity wheel_joint =
      model_.JointByName(_ecm, wheel_joint_name_);
  if (wheel_joint == ignition::gazebo::kNullEntity) {
    ignerr << wheel_joint_name_ << " not found" << std::endl;
    return;
  }

  ignition::gazebo::Entity base_joint =
      model_.JointByName(_ecm, base_joint_name_);
  if (base_joint == ignition::gazebo::kNullEntity) {
    ignerr << base_joint_name_ << " not found" << std::endl;
    return;
  }

  auto wheel_vel =
      _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
          wheel_joint);
  if (wheel_vel != nullptr) {
    *wheel_vel = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
  } else {
    _ecm.CreateComponent(
        wheel_joint,
        ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
  }

  auto base_pos =
      _ecm.Component<ignition::gazebo::components::JointPositionReset>(
          base_joint);
  if (base_pos != nullptr) {
    *base_pos = ignition::gazebo::components::JointPositionReset({base_pos_});
  } else {
    _ecm.CreateComponent(
        base_joint,
        ignition::gazebo::components::JointPositionReset({base_pos_}));
  }
}

void HolonomicTest::Update(const ignition::gazebo::UpdateInfo &_info,
                           ignition::gazebo::EntityComponentManager &_ecm) {
  (void)_info;
  (void)_ecm;
}

void HolonomicTest::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  (void)_info;
  (void)_ecm;
}

void HolonomicTest::CreateIgnitionIf(void) {
  this->node_.Subscribe("wheel_speed", &HolonomicTest::OnWheelSpeedMessage,
                        this);
  this->node_.Subscribe("base_pos", &HolonomicTest::OnBasePosMessage, this);
}

void HolonomicTest::OnWheelSpeedMessage(const ignition::msgs::Float &msg) {
  wheel_speed_ = msg.data();
}

void HolonomicTest::OnBasePosMessage(const ignition::msgs::Float &msg) {
  base_pos_ = msg.data();
}

} // namespace model_test

#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(model_test::HolonomicTest, ignition::gazebo::System,
                    model_test::HolonomicTest::ISystemConfigure,
                    model_test::HolonomicTest::ISystemPreUpdate,
                    model_test::HolonomicTest::ISystemUpdate,
                    model_test::HolonomicTest::ISystemPostUpdate)
