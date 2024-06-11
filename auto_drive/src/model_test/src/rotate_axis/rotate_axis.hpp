#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace model_test {
class RotateAxis : public ignition::gazebo::System,
                   public ignition::gazebo::ISystemConfigure,
                   public ignition::gazebo::ISystemPreUpdate,
                   public ignition::gazebo::ISystemUpdate,
                   public ignition::gazebo::ISystemPostUpdate {
public:
  RotateAxis();
  ~RotateAxis() override;
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;
  void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;
  void
  PostUpdate(const ignition::gazebo::UpdateInfo &_info,
             const ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  void CreateIgnitionIf(void);
  void OnWheelSpeedMessage(const ignition::msgs::Float &msg);
  void OnBaseSpeedMessage(const ignition::msgs::Float &msg);

  ignition::gazebo::Model model_;
  ignition::transport::Node node_;
  std::string wheel_joint_name_{""};
  float wheel_speed_{0.0f};
  std::string base_joint_name_{""};
  float base_speed_{0.0f};
};
} // namespace model_test