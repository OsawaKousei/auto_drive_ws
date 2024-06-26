import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtQuick.Extras 1.4
import "qrc:/qml"

Rectangle 
{
  Layout.minimumWidth: 400
  Layout.minimumHeight: 110

  Row {
    spacing: 5
    
    Button {
      text: "Forward"
      onPressed: { SwitchPanel.OnForwardButton(); }
    }

    Button {
      text: "WheelStop"
      onPressed: { SwitchPanel.OnWheelStopButton(); }
    }

    Button {
      text: "backward"
      onPressed: { SwitchPanel.OnBackwardButton(); }
    }

    Button {
      text: "Right"
      onPressed: { SwitchPanel.OnRightButton(); }
    }

    Button {
      text: "BaseStop"
      onPressed: { SwitchPanel.OnBaseStopButton(); }
    }

    Button {
      text: "Left"
      onPressed: { SwitchPanel.OnLeftButton(); }
    }
  }
}
