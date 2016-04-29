import QtQuick 2.6
import QtQuick.Controls 1.5
import QtQuick.Dialogs 1.2

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: qsTr("Hello World")
    property double dt: 0.02
    property variant fts: []

    signal doStep(double dt)

    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("&Open")
                onTriggered: console.log("Open action triggered");
            }
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
        }
    }

    Canvas {
        id: mycanvas
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d");
            ctx.fillStyle = Qt.rgba(0.8, 0.8, 1.0, 1);
            ctx.fillRect(0, 0, width, height);
            ctx.fillStyle = Qt.rgba(1.0, 0.0, 0.0, 1);
            for(var n in fts)
            {
                //ctx.save()
                //ctx.translate(ellipse.center.x, ellipse.center.y)
                //ctx.rotate((Math.PI / 180) * degrees)
                //ctx.translate(-ellipse.center.x, -ellipse.center.y)
                ctx.ellipse(100, 100, 20, 50);
                ctx.stroke();
                //ctx.restore()
                //ctx.fillRect(width / 2 + fts[n].x - 2, height / 2 - fts[n].y - 2, 5, 5);
            }
        }
    }
    Timer {
        id: timer
        interval: dt * 1000
        repeat: true
        running: true
        onTriggered: {
            doStep(dt);
        }
    }
    function onFeature(n, x, y, sxx, sxy, syy) {
        fts[n] = {x: x, y: y, sxx: sxx, sxy: sxy, syy: syy};
        mycanvas.requestPaint();
        /*var ctx = mycanvas.getContext("2d");
        var w = mycanvas.width, h = mycanvas.height;
        //var x = 10, y = 10;
        */
    }
    /*MainForm {
        anchors.fill: parent
        button1.onClicked: messageDialog.show(qsTr("Button 1 pressed"))
        button2.onClicked: messageDialog.show(qsTr("Button 2 pressed"))
    }*/

    MessageDialog {
        id: messageDialog
        title: qsTr("May I have your attention, please?")

        function show(caption) {
            messageDialog.text = caption;
            messageDialog.open();
        }
    }
}
