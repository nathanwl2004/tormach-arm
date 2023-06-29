import QtQuick 2.0
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import Machinekit.Controls 1.0
import Machinekit.HalRemote.Controls 1.0
import Machinekit.HalRemote 1.0

HalApplicationWindow {
  id: main
  property int digitalOutputs: 4
  property int digitalInputs: 4

  name: "io-rcomp"
  title: qsTr("HAL-IO")

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10

    Item {
      Layout.fillHeight: true
    }

    Repeater {
      model: main.digitalInputs

      HalButton {
        Layout.alignment: Layout.Center
        name: "digital_in_%1".arg(index + 1)
        text: "Digital In %1".arg(index + 1)
        halPin.direction: HalPin.IO
        checkable: true
      }
    }

    Repeater {
      model: main.digitalOutputs

      RowLayout {
        Layout.alignment: Layout.Center
        Label {
          text: "Digital Out %1: ".arg(index + 1)
        }
        HalLed {
          name: "digital_out_%1".arg(index + 1)
        }
      }
    }

    Item {
      Layout.fillHeight: true
    }
  }
}
