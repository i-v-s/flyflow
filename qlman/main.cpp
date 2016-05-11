#include <QApplication>
#include <QQmlApplicationEngine>
#include "kalmandraft.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    QObject * qml = engine.rootObjects().at(0);
    KalmanDraft kd;
    QObject::connect(qml, SIGNAL(doStep(double)),           &kd, SLOT(step(double)));
    QObject::connect(qml, SIGNAL(setAccel(double, double)), &kd, SLOT(setAccel(double,double)));
    QObject::connect(&kd, SIGNAL(onFeature(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)), qml, SLOT(onFeature(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
    QObject::connect(&kd, SIGNAL(onPos(QVariant, QVariant, QVariant)), qml, SLOT(onPos(QVariant, QVariant, QVariant)));
    return app.exec();
}
