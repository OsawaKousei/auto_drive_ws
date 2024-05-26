#include <ignition/gui/qt.h>
#include <ignition/transport/Node.hh>
#include <ignition/gui/Plugin.hh>

namespace model_test
{

class SwitchPanel : public ignition::gui::Plugin
{
  Q_OBJECT

public:
  SwitchPanel();
  virtual ~SwitchPanel();
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

protected slots:
  void OnForwardButton(void);
  void OnWheelStopButton(void);
  void OnBackwardButton(void);
  void OnRightButton(void);
  void OnBaseStopButton(void);
  void OnLeftButton(void);

private:
  void CreateIgnitionIf(void);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher wheel_speed_pub_;
  ignition::transport::Node::Publisher base_speed_pub_;
  float forward_speed_{1.0f};
  float reverse_speed_{-1.0f};
};

}
