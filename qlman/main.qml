import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Dialogs 1.2

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: qsTr("Hello World")
    property double dt: 0.02
    property variant fts: []
    property double px: 0
    property double py: 0
    property double pa: 0
    signal doStep(double dt)
    signal setAccel(double ax, double ay)
    function ellipse(ctx, cx, cy, rx, ry, a)
    {
        ctx.save()
        ctx.translate(cx, cy)
        ctx.rotate(-a)
        ctx.ellipse(-rx, -ry, rx * 2, ry * 2)
        ctx.stroke()
        ctx.restore()
    }

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
            var w2 = width / 2, h2 = height / 2;
            var ctx = getContext("2d");
            ctx.fillStyle = Qt.rgba(0.8, 0.8, 1.0, 1);
            ctx.fillRect(0, 0, width, height);
            var tx = px, ty = py;
            ctx.fillStyle = Qt.rgba(1.0, 0.0, 0.0, 1);
            ctx.fillRect(w2 + tx - 2, h2 - ty - 2, 5, 5);

            //ellipse(ctx, w2, h2, 20, 40, 3.14159 / 4);
            for(var n in fts)
            {
                var f = fts[n];
                ellipse(ctx, w2 + tx + f.x, h2 - f.y - ty, f.radx, f.rady, f.a)
            }

            ctx.fillStyle = Qt.rgba(0.0, 0.0, 0.0, 1);
            ctx.beginPath();
            for(var n in fts)
            {
                f = fts[n];
                ctx.ellipse(w2 + f.rx, h2 - f.ry, 5, 5);
            }
            ctx.fill();
        }
        MouseArea {
            anchors.fill: parent
            property double ox: 0;
            property double oy: 0;
            onPositionChanged: {
                var w2 = width / 2, h2 = height / 2;
                var kp = 2, kd = 10;
                var tx = px, ty = py;
                var ax = (mouse.x - tx - w2) * kp;
                var ay = (h2 - ty - mouse.y) * kp;
                ax += (ox - tx) * kd;
                ay += (oy - ty) * kd;
                setAccel(ax, ay);
                ox = tx;
                oy = ty;
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
    function onFeature(n, x, y, a, radx, rady, rx, ry) {

        fts[n] = {x: x, y: y, a: a, radx: radx, rady: rady, rx: rx, ry: ry};
        mycanvas.requestPaint();
        /*var ctx = mycanvas.getContext("2d");
        var w = mycanvas.width, h = mycanvas.height;
        //var x = 10, y = 10;
        */
    }
    function onPos(tx, ty, ta)
    {
        px = tx;
        py = ty;
        pa = ta;
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
