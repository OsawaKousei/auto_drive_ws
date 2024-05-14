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
      text: "go"
      onPressed: { HolonomicPanel.OnGoButton(); }
    }

    Button {
      text: "stop"
      onPressed: { HolonomicPanel.OnStopButton(); }
    }

    Button {
      text: "Forward"
      onPressed: { HolonomicPanel.OnForwardButton(); }
    }

    Button {
      text: "backward"
      onPressed: { HolonomicPanel.OnBackwardButton(); }
    }

    Button {
      text: "Right"
      onPressed: { HolonomicPanel.OnRightButton(); }
    }

    Button {
      text: "Left"
      onPressed: { HolonomicPanel.OnLeftButton(); }
    }
  }
}
