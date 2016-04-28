#ifndef KALMAN_H
#define KALMAN_H
#include <Eigen/Core>
#include <Eigen/LU>

template<class T, int n>
class Kalman
{
public:
    typedef Eigen::Matrix<T, n, n> Matrix;
    typedef Eigen::Matrix<T, n, 1> Vector;
	Matrix G_; // Матрица предсказания 
	Matrix Q_; // Ковариация (шум) процесса
    Vector X_;
    Matrix Sigma_; // Текущая ковариация процесса
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

template<class T, int n, int k>
class Corrector
{
public:
    Eigen::Matrix<T, k, n> H_;
    Eigen::Matrix<T, k, k> R_;
    Corrector()
    {
        R_.setZero();
    }
    virtual bool h(Eigen::Matrix<T, k, 1> * Z, Eigen::Matrix<T, k, n> * H_, const Eigen::Matrix<T, n, 1> & X)
    {
        *Z = *H_ * X;
        return true;
    }

    void correct(Eigen::Matrix<T, n, 1> & x, Eigen::Matrix<T, n, n> & sigma, Eigen::Matrix<T, k, 1> z)
    {
        Eigen::Matrix<T, k, 1> zt;
        if(!h(&zt, &H_, x)) return;

        Eigen::Matrix<T, k, k> S = H_ * sigma * H_.transpose() + R_;
        Eigen::Matrix<T, n, k> K = sigma * H_.transpose() * S.inverse();
        x += K * (z - zt);
        sigma = (Eigen::Matrix<T, n, n>::Identity() - K * H_) * sigma;
    }
};


#endif // KALMAN_H
