#ifndef KALMAN_H
#define KALMAN_H
#include <eigen3/Eigen/Core>

template<class T, int n>
class Kalman
{
public:
    typedef Eigen::Matrix<T, n, n> Matrix;
    typedef Eigen::Matrix<T, n, 1> Vector;
    Matrix G_, Q_;
    Vector X_;
    Matrix Sigma_;
    Kalman()
    {
        G_.setIdentity();
        Q_.setZero();
        Sigma_.setZero();
    }

    virtual void g(Vector & newX, const Vector & X)
    {
        newX = G_ * X;
    }
    void predict()
    {
        Vector X = X_;
        g(X_, X);
        Sigma_ = G_ * Sigma_ * G_.transpose() + Q_;
    }
};

template<class T, class H, int n, int k>
class Corrector
{
public:
    Eigen::Matrix<T, k, n> H_;
    Eigen::Matrix<T, k, k> R_;
    Corrector()
    {
        R_.setZero();
    }
    /*virtual bool h(Eigen::Matrix<T, k, 1> & Z, const Eigen::Matrix<T, n, 1> & X)
    {
        Z = H_ * X;
        return true;
    }*/

    void correct(Eigen::Matrix<T, n, 1> & x, Eigen::Matrix<T, n, n> & sigma, Eigen::Matrix<T, k, 1> z, H * h)
    {
        Eigen::Matrix<T, k, 1> zt;
        if(!h->h(&zt, &H_, x)) return;

        Eigen::Matrix<T, k, k> t = H_ * sigma * H_.transpose() + R_;
        Eigen::Matrix<T, n, k> K = sigma * H_.transpose() * t.inverse();
        x += K * (z - zt);
        Eigen::Matrix<T, n, n> I();
        sigma = (Eigen::Matrix<T, n, n>::Identity() - K * H_) * sigma;
    }
};


#endif // KALMAN_H
