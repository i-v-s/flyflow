#ifndef KALMANCOPTER_H
#define KALMANCOPTER_H
#include "kalman.h"

// State:
    // Quaternion att
    // Vector<3> pos
    // Vector<3> vel

class KalmanCopter: public Kalman<double, 10>
{
    double time_, dt_;
public:
    KalmanCopter(): time_(0), dt_(0)
    {
        G_ = Eigen::Matrix<double, 10, 10>::Identity();
        G_.block<3, 3>(4, 7) = Eigen::Matrix<double, 3, 3>::Identity(); // Velocity
    }
    Eigen::Vector3d pw_;
    void g(Vector & newX, const Vector & X)
    {
        newX = G_ * X;
        //Eigen::Map<const Eigen::Quaterniond>(X_.data()).w = 1;
        X_(3) = 1.0;
        //newX.block<3, 1>(7, 0) += a_ * q;

    }
    void predictByImu(double time, const Eigen::Vector3d & a, const Eigen::Vector3d & w)
    {
        double dt = time - time_;
        if(!time_)
        {
            time_ = time;
            pw_ = w;
            return;
        }
        Eigen::Vector3d sw = (pw_ + w) * 0.5 * dt;
        pw_ = w;
        time_ = time;
        double wx = sw(0), wy = sw(1), wz = sw(2);
        Eigen::Matrix<double, 4, 4> gq;// = G_.block<4, 4>(0, 0);
        gq << 1.0,  wz, -wy,  wx,
              -wz, 1.0,  wx,  wy,
               wy, -wx, 1.0,  wz,
              -wx, -wy, -wz, 1.0;
        G_.block<4, 4>(0, 0) = gq;
        //G_.block<


        Kalman::predict();
        Eigen::Map<Eigen::Quaterniond> q = Eigen::Map<Eigen::Quaterniond>(X_.data());
        q.normalize();
    }
    Eigen::Quaterniond getAttitude() const
    {
        return Eigen::Map<const Eigen::Quaterniond>(X_.data());
    }
};



#endif // KALMANCOPTER_H
