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
    property double pra: 0
    property double pda: 0
    property double mx: 0
    property double my: 0
    signal doStep(double dt)
    signal setAccel(double ax, double ay)
    function ellipse(ctx, cx, cy, rx, ry, a)
    {
        ctx.save()
        ctx.translate(cx, cy)
        ctx.rotate(-a)
        ctx.strokeStyle = Qt.rgba(0.0, 0.0, 0.0, 1)
        ctx.beginPath()
        ctx.ellipse(-rx, -ry, rx * 2, ry * 2)
        ctx.stroke()
        ctx.restore()
    }
    function drawPos(ctx, x, y, ra, a, da)
    {
        ctx.save()
        ctx.translate(x, y)
        ctx.rotate(-ra)
        ctx.strokeStyle = Qt.rgba(0.0, 1.0, 0.0, 1)
        ctx.beginPath()
        ctx.moveTo(0, 0)
        ctx.lineTo(20, 0)
        ctx.stroke()
        ctx.restore()

        ctx.save()
        ctx.translate(x, y)
        ctx.rotate(-a)
        ctx.strokeStyle = Qt.rgba(1.0, 0.0, 0.0, 1)
        ctx.beginPath()
        //ctx.moveTo(0, 0)
        ctx.arc(0, 0, 20, -da, da)
        //ctx.lineTo(0, 0)
        //ctx.fill()
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
            //ctx.fillRect(w2 + tx - 2, h2 - ty - 2, 5, 5);
            drawPos(ctx, w2 + tx, h2 - ty, pra, pa, pda)

            //ellipse(ctx, w2, h2, 20, 40, 3.14159 / 4);
            ctx.beginPath()
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
            onPositionChanged: {
                mx = mouse.x
                my = mouse.y
            }
            onReleased: {
                mx = 0
                my = 0
            }
        }
    }
    Timer {
        id: timer
        interval: dt * 1000
        repeat: true
        running: true
        property double ox: 0;
        property double oy: 0;
        onTriggered: {
            var kp = 2, kd = 10;
            var tx = px, ty = py;
            var ax = (ox - tx) * kd;
            var ay = (oy - ty) * kd;
            if(mx || my)
            {
                var w2 = width / 2, h2 = height / 2;
                ax += (mx - tx - w2) * kp;
                ay += (h2 - ty - my) * kp;
            }
            setAccel(ax, ay);
            ox = tx;
            oy = ty;
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
    function onPos(tx, ty, ra, ta, da)
    {
        px = tx;
        py = ty;
        pa = ta;
        pra = ra;
        pda = da
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
