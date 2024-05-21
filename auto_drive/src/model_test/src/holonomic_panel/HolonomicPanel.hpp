#include <ignition/gui/qt.h>
#include <ignition/transport/Node.hh>
#include <ignition/gui/Plugin.hh>

namespace model_test
{

class HolonomicPanel : public ignition::gui::Plugin
{
  Q_OBJECT

public:
  HolonomicPanel();
  virtual ~HolonomicPanel();
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

protected slots:
  void OnGoButton(void);
  void OnStopButton(void);
  void OnForwardButton(void);
  void OnBackwardButton(void);
  void OnRightButton(void);
  void OnLeftButton(void);

private:
  void CreateIgnitionIf(void);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher wheel_speed_pub_;
  ignition::transport::Node::Publisher base_pos_pub_;
  float wheel_speed_{1.0f};
};

}  // namespace iginition_plugin_lecture
