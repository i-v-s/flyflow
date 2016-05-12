#ifndef KALMAN_H
#define KALMAN_H
#include <Eigen/Core>
#include <Eigen/LU>

template<class T, int n, int k, bool test = false>
class Kalman
{
public:
    typedef Eigen::Matrix<T, n, n> Matrix;
    typedef Eigen::Matrix<T, k, n> HMatrix;
    typedef Eigen::Matrix<T, n, 1> Vector;
	Matrix G_; // Матрица предсказания 
	Matrix Q_; // Ковариация (шум) процесса
    Vector X_; // Вектор состояния
    Matrix Sigma_; // Текущая ковариация процесса
    HMatrix H_; // Матрица измерения
    Eigen::Matrix<T, k, k> R_; // Ковариация (шум) измерения
    Kalman()
    {
        G_.setIdentity();
        Q_.setZero();
        Sigma_.setZero();
        R_.setZero();
    }
    virtual void g(Vector & newX)
    {
        newX = G_ * X_;
    }
	void calcG(Matrix & G, double dx = 0.01) // Примерная линеаризация
	{
		Matrix oG = G_;
		Vector nx, nx1, ox = X_;
		g(nx);
		for (int x = 0; x < n; x++)
		{
			X_(x) += dx;
			g(nx1);
            G.template block<n, 1>(0, x) = (nx1 - nx) / dx;
			X_ = ox;
		}
		G_ = oG;
	}
    void predict()
    {
        Vector X;
        g(X);
		X_ = X;
        Sigma_ = G_ * Sigma_ * G_.transpose() + Q_;
    }
    virtual bool h(Eigen::Matrix<T, k, 1> & z)
    {
        z = H_ * X_;
        return true;
    }
    void calcH(HMatrix & H, T dx = 0.01)
	{
        Eigen::Matrix<T, k, n> oH = H_;
		Vector ox = X_;
        Eigen::Matrix<T, k, 1> nz, nz1;
		h(nz);
		for (int x = 0; x < n; x++)
		{
			X_(x) += dx;
			h(nz1);
            H.template block<k, 1>(0, x) = (nz1 - nz) / dx;
			X_ = ox;
		}
        H_ = oH;
	}
    void correct(Eigen::Matrix<T, k, 1> z)
    {
        Eigen::Matrix<T, k, 1> zt;
        if(!h(zt)) return;
        if(test)
        {
            Eigen::Matrix<T, k, n> H, H1 = H_;
            calcH(H);
            for(int x = 0; x < n; x++) for(int y = 0; y < k; y++) assert(abs(H1(y, x) - H(y, x)) < 1E-3);
        }

        Eigen::Matrix<T, k, k> S = H_ * Sigma_ * H_.transpose() + R_;
        Eigen::Matrix<T, n, k> K = Sigma_ * H_.transpose() * S.inverse();
        X_ += K * (z - zt);
        Sigma_ = (Eigen::Matrix<T, n, n>::Identity() - K * H_) * Sigma_;
    }
};


#endif // KALMAN_H
