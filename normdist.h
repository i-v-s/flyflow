#pragma once
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <vector>

double rand_norm();

template<class T, int N>
class NormalDistribution
{
public:
	typedef Eigen::Matrix<T, N, N> Matrix;
	typedef Eigen::Matrix<T, N, 1> Vector;
	Vector mean;
	Matrix sigma, A;
	void calcA() { A = sigma.llt().matrixL();}
	NormalDistribution(const std::vector<Eigen::Matrix<T, N, 1> > & data) 
	{
		Vector sum;
		sum.setZero();
		Matrix sum2;
		sum2.setZero();
		for (auto d = data.begin(); d != data.end(); d++)
		{
			sum += *d;
			sum2 += *d * d->transpose();
		}
		int len = data.size();
		sum /= len;
		sum2 /= len;
		mean = sum;
		sigma = sum2 - sum * sum.transpose();
		calcA();
	}
	NormalDistribution(const Vector & mean, const Matrix & sigma) : mean(mean), sigma(sigma)
	{
		calcA();
	}
	void generate(Vector & out)
	{
		Vector t;
		for (int x = 0; x < N; x++) t(x) = rand_norm();
		out = A * t + mean;
	}
	void ellipse(double sigmaCount)
	{

	}
};
