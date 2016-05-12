#ifndef KALMANDRAFT_H
#define KALMANDRAFT_H
#include "../kalman.h"
#include <QObject>
#include <QVariant>


template<int FC>
class KalmanFeatures2D : public Kalman<double, 4 + FC * 2, FC> // линейные скорости, угол, угловая скорость, две координаты на фичу
        // Координаты фич относительно робота, без учёта поворота
{
public:
    double dt;
    KalmanFeatures2D(double dt = 0.02): dt(dt)
    {
        for(int x = 0; x < this->Sigma_.rows(); x++)
            this->Sigma_(x, x) = 0.3;
        this->H_.setZero();
        this->X_.setZero();
        this->G_(2, 3) = dt; // Крутим

        this->Q_(0, 0) = 0.1; // Шум скорости
        this->Q_(1, 1) = 0.1; //
        for(int x = 0; x < FC; x++) // Движение фич
        {
            this->G_(4 + x * 2, 0) = -dt;
            this->G_(5 + x * 2, 1) = -dt;
            this->X_(4 + x * 2) = ((double)rand() / RAND_MAX) * 200.0 - 100.0;
            this->X_(5 + x * 2) = ((double)rand() / RAND_MAX) * 200.0 - 100.0;
            this->Sigma_(4 + x * 2, 4 + x * 2) = 10000; // Неизвестно, где они
            this->Sigma_(5 + x * 2, 5 + x * 2) = 10000;
            this->R_(x, x) = 0.01; // Шум измерения
            //int t = 4 + x * 2;
            //this->Q_(t, t) = 0.05; // Шум фич
            //this->Q_(t + 1, t + 1) = 0.05;
        }
    }
    bool h(Eigen::Matrix<double, FC, 1> & z)
    {
        for(int f = 0; f < FC; f++)
        {
            double x = this->X_(4 + f * 2), y = this->X_(5 + f * 2), d = y / x;
            z(f) = atan2(y, x) + this->X_(2);
            this->H_(f, 2) = 1;
            this->H_(f, 4 + f * 2) = -y / ((1 + d * d) * x * x); // (atan x)' = 1 / (1 + x^2)
            this->H_(f, 5 + f * 2) =  1 / ((1 + d * d) * x);
        }
        return true;
    }
};

class KalmanDraft : public QObject
{
    Q_OBJECT
    KalmanFeatures2D<4> k; // x1, y1, a
    Eigen::Vector2d pos, vel, acc; // Реальные позиция, скорость, ускорение
    double a;
    Eigen::Vector2d fts[4];
public:
    explicit KalmanDraft(QObject *parent = 0);
signals:
    void onFeature(QVariant num, QVariant x, QVariant y, QVariant a, QVariant radx, QVariant rady, QVariant rx, QVariant ry);
    void onPos(QVariant x, QVariant y, QVariant a);
public slots:
    void step(double dt);
    void setAccel(double ax, double ay);
};

#endif // KALMANDRAFT_H
