#ifndef KALMAN_H
#define KALMAN_H
#include <Eigen/Core>
#include <Eigen/LU>

template<class T, int n, int k>
class Kalman
{
public:
    typedef Eigen::Matrix<T, n, n> Matrix;
    typedef Eigen::Matrix<T, n, 1> Vector;
	Matrix G_; // Матрица предсказания 
	Matrix Q_; // Ковариация (шум) процесса
    Vector X_; // Вектор состояния
    Matrix Sigma_; // Текущая ковариация процесса
    Eigen::Matrix<T, k, n> H_; // Матрица измерения
    Eigen::Matrix<T, k, k> R_; // Ковариация (шум) измерения
    Kalman()
    {
        G_.setIdentity();
        Q_.setZero();
        Sigma_.setZero();
        R_.setZero();
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
    virtual bool h(Eigen::Matrix<T, k, 1> * Z, Eigen::Matrix<T, k, n> * H_, const Eigen::Matrix<T, n, 1> & X)
    {
        *Z = *H_ * X;
        return true;
    }
    void correct(Eigen::Matrix<T, k, 1> z)
    {
        Eigen::Matrix<T, k, 1> zt;
        if(!h(&zt, &H_, X_)) return;

        Eigen::Matrix<T, k, k> S = H_ * Sigma_ * H_.transpose() + R_;
        Eigen::Matrix<T, n, k> K = Sigma_ * H_.transpose() * S.inverse();
        X_ += K * (z - zt);
        Sigma_ = (Eigen::Matrix<T, n, n>::Identity() - K * H_) * Sigma_;
    }
};


#endif // KALMAN_H
