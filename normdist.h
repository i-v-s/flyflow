#pragma once
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <vector>

double rand_norm();

struct NDEllipse
{
	double alpha, rx, ry;
	double cx, cy;
	bool test(double x, double y) const;
};

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
	void generate(Vector & out) const
	{
		Vector t;
		for (int x = 0; x < N; x++) t(x) = rand_norm();
		out = A * t + mean;
	}
	NDEllipse ellipse(double dev, int x = 0, int y = 1) const
	{
		Matrix inv = sigma.inverse();
		double alpha = atan2(sigma(0, 1) + sigma(1, 0), sigma(0, 0) - sigma(1, 1)) / 2;
		double i00 = inv(0, 0), i11 = inv(1, 1), i01 = inv(1, 0) + inv(0, 1);
		double a = sin(alpha), b = cos(alpha);
		double rx = dev / sqrt(i00 * b * b + i01 * a * b + i11 * a * a);
		double ry = dev / sqrt(i00 * a * a - i01 * a * b + i11 * b * b);
		return {alpha, rx, ry, mean(x), mean(y)};
	}
};
