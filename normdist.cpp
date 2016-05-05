#include "normdist.h"
#include <gtest/gtest.h>

TEST(NormalDistributionTest, analize)
{
	std::vector<Eigen::Vector2d> v;
	for (int x = 0; x < 1000; x++)
	{
		Eigen::Matrix<double, 2, 1> m;
		m << sin(x), cos(x);
		v.push_back(m);
	}
	NormalDistribution<double, 2> nd(v);
	EXPECT_NEAR(nd.mean(0), 0, 0.001);
	EXPECT_NEAR(nd.mean(1), 0, 0.001);
	EXPECT_NEAR(nd.sigma(0, 0), 0.5, 0.001);
	EXPECT_NEAR(nd.sigma(0, 1), 0.0, 0.001);
	EXPECT_NEAR(nd.sigma(1, 0), 0.0, 0.001);
	EXPECT_NEAR(nd.sigma(1, 1), 0.5, 0.001);
}

double rnd(double min, double max)
{
	return ((double)rand() / RAND_MAX) * (max - min) + min;
}

//Подпрограмма нормального распределения - возв СВ по гауссу с МО 0 и дисп 1.                
double rand_norm()
{
	static double r;
	static bool flag = false;
	if (flag) { flag = false; return(r); }
	while (1)
	{
		double v1 = rnd(-1, 1), v2 = rnd(-1, 1), s = v1 * v1 + v2 * v2;
		if (s <= 1.0 && s > 0.0)
		{
			r = v1 * sqrt(-2.0 * log(s) / s); //за один раз получаем 2 СВ
			flag = 1;
			return v2 * sqrt(-2.0 * log(s) / s);
		}
	}
}

TEST(NormalDistributionTest, generate)
{
	Eigen::Matrix2d s;
	Eigen::Vector2d m;
	m << 1, 2;
	s << 1.0, 0.5,
		 0.5, 1.0;
	NormalDistribution<double, 2> nd(m, s);
	std::vector<Eigen::Vector2d> v;
	for (int x = 0; x < 10000; x++)
	{
		Eigen::Vector2d m;
		nd.generate(m);
		v.push_back(m);
	}
	NormalDistribution<double, 2> nd2(v);
	EXPECT_NEAR(nd.mean(0), nd2.mean(0), 0.02);
	EXPECT_NEAR(nd.mean(1), nd2.mean(1), 0.02);
	EXPECT_NEAR(nd.sigma(0, 0), nd2.sigma(0, 0), 0.02);
	EXPECT_NEAR(nd.sigma(0, 1), nd2.sigma(0, 1), 0.02);
	EXPECT_NEAR(nd.sigma(1, 0), nd2.sigma(1, 0), 0.02);
	EXPECT_NEAR(nd.sigma(1, 1), nd2.sigma(1, 1), 0.02);
}
