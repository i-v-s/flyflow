#ifndef KALMANDRAFT_H
#define KALMANDRAFT_H
#include "../kalman.h"
#include <QObject>
#include <QVariant>


template<int FC>
class KalmanFeatures2D : public Kalman<double, 4 + FC * 2> // линейные скорости, угол, угловая скорость, две координаты на фичу
        // Координаты фич относительно робота, без учёта поворота
{
public:
    double dt;
    KalmanFeatures2D(double dt = 0.02): dt(dt)
    {
        this->X_.setZero();
        this->G_(2, 3) = dt; // Крутим
        for(int x = 0; x < FC; x++) // Движение фич
        {
            this->G_(4 + x * 2, 0) = -dt;
            this->G_(5 + x * 2, 1) = -dt;
            this->Sigma_(4 + x * 2, 4 + x * 2) = 100; // Неизвестно, где они
            this->Sigma_(4 + x * 2, 4 + x * 2) = 100;

        }
    }
    /*void g(typename Kalman<double, 4 + FC * 2>::Vector & newX, const typename Kalman<double, 3 + FC * 2>::Vector & X)
    {
        // Крутим
        newX(2) = X(2) + X(3) * dt;
        for(int x = 0; x < FC; x++) // Двигаем фичи
        {
            newX(4 + x * 2) = X(4 + x * 2) - X(0) * dt;
            newX(5 + x * 2) = X(5 + x * 2) - X(1) * dt;
        }
    }*/
};


class KalmanDraft : public QObject
{
    Q_OBJECT
    //Q_PROPERTY( name READ name WRITE setName NOTIFY nameChanged)
    KalmanFeatures2D<2> k; // x1, y1, a
public:
    explicit KalmanDraft(QObject *parent = 0);

signals:
    void onFeature(QVariant num, QVariant x, QVariant y, QVariant sxx, QVariant sxy, QVariant syy);
public slots:
    void step(double dt);
};

#endif // KALMANDRAFT_H
