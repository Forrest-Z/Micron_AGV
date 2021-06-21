import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4

Rectangle  {
    id: root
    x: 0
    width: 570
    height: 550
    color: "#ffffff"
    z: -100

//    TurnIndicator {
//        id: leftIndicator
//        x: 101
//        anchors.verticalCenter: parent.verticalCenter
//        width: 40
//        height: 40
//        anchors.verticalCenterOffset: 174

//        direction: Qt.LeftArrow
//        on: true //valueSource.turnSignal === Qt.LeftArrow
//    }

//    TurnIndicator {
//        id: rightIndicator
//        x: 431
//        anchors.verticalCenter: parent.verticalCenter
//        anchors.right: parent.right
//        width: 40
//        height: 40
//        anchors.verticalCenterOffset: 174
//        anchors.rightMargin: 99
//        direction: Qt.RightArrow
//        on: true //valueSource.turnSignal === Qt.RightArrow
//    }

    CircularGauge {
        id: speedgauge
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        width: 320//320
        height: width
        anchors.verticalCenterOffset: -24
        anchors.horizontalCenterOffset: 0
        maximumValue: 32
        minimumValue: -1.2
        value: qspeed.m_speed
        style: CircularGaugeStyle {
            tickmarkInset: toPixels(-2.0)//(0.04)
            minorTickmarkInset: tickmarkInset
            labelStepSize: 5
            labelInset: toPixels(-2.0)//(0.23)

            property real xCenter: outerRadius
            property real yCenter: outerRadius
            property real needleLength: toPixels(0.80) //outerRadius - tickmarkInset * 1.25
            property real needleTipWidth: toPixels(0.02)
            property real needleBaseWidth: toPixels(0.06)
            property bool halfGauge: false

            function toPixels(percentage) {
                return percentage * outerRadius;
            }

            function degToRad(degrees) {
                return degrees * (Math.PI / 180);
            }

            function radToDeg(radians) {
                return radians * (180 / Math.PI);
            }

            background: Canvas {
                onPaint: {
                    var ctx = getContext("2d");
                    ctx.reset();
                    //paintBackground(ctx);
                }

                Image {
                    id: circular_background
                    x: 10
                    y: -12
                    anchors.centerIn: speedgauge
                    width: 300
                    height: width
                    anchors.horizontalCenter: speedgauge.horizontalCenter
                    anchors.verticalCenter: speedgauge.verticalCenter
                    z: 0
                    source: "../icons/new/speedometer_2.png"
                    fillMode: Image.PreserveAspectFit
                    visible: true
                }

                Text {
                    id: speedText
                    font.pixelSize: toPixels(0.6)
                    font.bold: true
                    text: kphInt
                    color: "orange"
                    horizontalAlignment: Text.AlignRight
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    //anchors.topMargin: toPixels(0.1)
                    readonly property int kphInt: control.value
                }
                Text {
                    text: "km/h"
                    color: "orange"
                    font.pixelSize: toPixels(0.15)
                    anchors.top: speedText.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                }


            }

            needle: Canvas {
                implicitWidth: needleBaseWidth
                implicitHeight: needleLength

                property real xCenter: width / 2
                property real yCenter: height / 2
                property real needle_height: height

//                onPaint: {
//                    var ctx = getContext("2d");
//                    ctx.reset();
//                    ctx.beginPath();
//                    ctx.moveTo(xCenter, needle_height);
//                    ctx.lineTo(xCenter - needleBaseWidth / 2, needle_height - needleBaseWidth / 2);
//                    ctx.lineTo(xCenter - needleTipWidth / 2, 0);
//                    ctx.lineTo(xCenter, yCenter - needleLength);
//                    ctx.lineTo(xCenter, 0);
//                    ctx.closePath();
//                    ctx.fillStyle = Qt.rgba(0.66, 0, 0, 0.66);
//                    ctx.fill();

//                    ctx.beginPath();
//                    ctx.moveTo(xCenter, needle_height)
//                    ctx.lineTo(width, needle_height - needleBaseWidth / 2);
//                    ctx.lineTo(xCenter + needleTipWidth / 2, 0);
//                    ctx.lineTo(xCenter, 0);
//                    ctx.closePath();
//                    ctx.fillStyle = Qt.lighter(Qt.rgba(0.66, 0, 0, 0.66));
//                    ctx.fill();
//                }

                Image {
                    id: pointer
                    source: "../icons/new/pointer_2.png"
                    x: -27
                    y: -16
                    anchors.centerIn: speedgauge
                    width: 62
                    height: 62
                    anchors.horizontalCenter: speedgauge.horizontalCenter
                    anchors.verticalCenter: speedgauge.verticalCenter
                    anchors.verticalCenterOffset: 0
                    anchors.horizontalCenterOffset: 0
                    z: 0
                    fillMode: Image.PreserveAspectFit
                    visible: true
                }

            }


            foreground: null
        }
    }


    CircularGauge {
        id: steeringGauge
        anchors.verticalCenter: speedgauge.verticalCenter
        anchors.horizontalCenter: speedgauge.horizontalCenter
        width: 140//320
        height: 140
        anchors.verticalCenterOffset: 190
        anchors.horizontalCenterOffset: 0
        z: 1
        maximumValue: 25
        minimumValue: -25
        // set cmd value
        value: qangle.m_angle
        style: CircularGaugeStyle {
            tickmarkInset: toPixels(-6.0)
            minorTickmarkInset: tickmarkInset
            labelStepSize: 5
            labelInset: toPixels(-6.0)

            property real xCenter: outerRadius
            property real yCenter: outerRadius
            property real needleLength: outerRadius - tickmarkInset * 1.25
            property real needleTipWidth: toPixels(0.02)
            property real needleBaseWidth: toPixels(0.06)
            property bool halfGauge: false

            function toPixels(percentage) {
                return percentage * outerRadius;
            }

            function degToRad(degrees) {
                return degrees * (Math.PI / 180);
            }

            function radToDeg(radians) {
                return radians * (180 / Math.PI);
            }

            needle: Canvas {
                implicitWidth: needleBaseWidth
                implicitHeight: needleLength
                property real xCenter: width / 2
                property real yCenter: height / 2

                Image {
                    id: steering_logo
                    x: -72
                    y: 520
                    width: 150
                    height: 150
                    source: "../icons/steering-wheel.png"
                    fillMode: Image.PreserveAspectFit
                    visible: true
                }
            }// end of needle
            foreground: null
        }
    }

    Image {
        id: auto
        anchors.centerIn: speedgauge
        width: 120
        height: 80
        anchors.verticalCenterOffset: -180
        anchors.horizontalCenterOffset: 0
        z: 0
        source: "../icons/new/Auto_1.png"
        fillMode: Image.PreserveAspectFit

        visible: on
        property bool on
        on:{
        if(qmode.m_current_mode == 1) return true //0 for brake mode, 1 for autonomous mode, 2 for manual mode
        else return false
        }
    }

    Image {
        id: manual
        anchors.centerIn: auto
        width: 160
        height: 80
        anchors.verticalCenterOffset: 0
        anchors.horizontalCenterOffset: 0
        z: 0
        source: "../icons/new/Manual_1.png"
        fillMode: Image.PreserveAspectFit

        visible: on
        property bool on
        on:{
        if(qmode.m_current_mode == 2) return true //0 for brake mode, 1 for autonomous mode, 2 for manual mode
        else return false
        }
    }

    Image {
        id: park
        anchors.centerIn: auto
        width: 160
        height: 80
        anchors.verticalCenterOffset: 0
        anchors.horizontalCenterOffset: 0
        z: 0
        source: "../icons/new/Park_1.png"
        fillMode: Image.PreserveAspectFit

        visible: on
        property bool on
        on:{
        if(qmode.m_current_mode == 0) return true //0 for brake mode, 1 for autonomous mode, 2 for manual mode
        else return false
        }
    }

    Text {
        id: mode
        anchors.top: speedgauge.top
        anchors.horizontalCenter: speedgauge.horizontalCenter
        anchors.horizontalCenterOffset: 102
        width: 71
        height: 22
        color: "#323232"
        text: qsTr("Mode")
        anchors.topMargin: -31
        font.bold: true
        styleColor: "#000000"
        font.pixelSize: 20
        visible: true
    }

    Image {
        id: bar_0
        anchors.centerIn: speedgauge
        width: 400
        height: width
        z: 0
        source: "../icons/new/bars_0.png"
        fillMode: Image.PreserveAspectFit
        visible: true
    }

    Image {
        id: bar_soft_brake
        anchors.centerIn: bar_0
        width: bar_0.width
        height: width
        z: 0
        source: "../icons/new/bars_brake_2.png"
        fillMode: Image.PreserveAspectFit
        visible: on
        property bool on
        on:{
        if(qbrake.m_brake == 1) return true
        else return false
        }
    }

    Image {
        id: bar_hard_brake
        anchors.centerIn: bar_0
        width: bar_0.width
        height: width
        z: 0
        source: "../icons/new/bars_brake_4.png"
        fillMode: Image.PreserveAspectFit
        visible: on
        property bool on
        on:{
        if(qbrake.m_brake == 2) return true
        else return false
        }
    }

    Image {
        id: bar_accelerate_1
        anchors.centerIn: bar_0
        width: bar_0.width
        height: width
        z: 0
        source: "../icons/new/bars_accelerate_2.png"
        fillMode: Image.PreserveAspectFit
        visible: on
        property bool on
        on:{
        if(qacceleration.m_acceleration_level == 1) return true
        else return false
        }
    }

    Image {
        id: bar_accelerate_2
        anchors.centerIn: bar_0
        width: bar_0.width
        height: width
        z: 0
        source: "../icons/new/bars_accelerate_3.png"
        fillMode: Image.PreserveAspectFit
        visible: on
        property bool on
        on:{
        if(qacceleration.m_acceleration_level == 2 || qacceleration.m_acceleration_level == 3) return true
        else return false
        }
    }

    Image {
        id: bar_accelerate_3
        anchors.centerIn: bar_0
        width: bar_0.width
        height: width
        z: 0
        source: "../icons/new/bars_accelerate_5.png"
        fillMode: Image.PreserveAspectFit
        visible: on
        property bool on
        on:{
            if(qacceleration.m_acceleration_level == 4 || qacceleration.m_acceleration_level == 5) return true
            else return false
        }
    }

//    Image {
//        id: brake_0
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/brake_0.png"
//        fillMode: Image.PreserveAspectFit

//        visible: true
//    }

//    Image {
//        id: brake_3
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/brake_3.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qbrake.m_brake == 1) return true
//        else return false
//        }
//    }

//    Image {
//        id: brake_5
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/brake_5.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qbrake.m_brake == 2) return true
//        else return false
//        }
//    }

//    Image {
//        id: accelerator_0
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/accelerator_0.png"
//        fillMode: Image.PreserveAspectFit
//        visible: true
//    }

//    Image {
//        id: accelerator_1
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/accelerator_1.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qacceleration.m_acceleration_level == 1) return true
//        else return false
//        }
//    }

//    Image {
//        id: accelerator_2
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/accelerator_2.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qacceleration.m_acceleration_level == 2) return true
//        else return false
//        }
//    }

//    Image {
//        id: accelerator_3
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/accelerator_3.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qacceleration.m_acceleration_level == 3) return true
//        else return false
//        }
//    }

//    Image {
//        id: accelerator_4
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/accelerator_4.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qacceleration.m_acceleration_level == 4) return true
//        else return false
//        }
//    }

//    Image {
//        id: accelerator_5
//        anchors.centerIn: speedgauge
//        width: 500
//        height: 500
//        z: 0
//        source: "../icons/new/accelerator_5.png"
//        fillMode: Image.PreserveAspectFit
//        visible: on

//        property bool on
//        on:{
//        if(qacceleration.m_acceleration_level == 5) return true
//        else return false
//        }
//    }

}

















































/*##^## Designer {
    D{i:0;height:0;width:0}
}
 ##^##*/
