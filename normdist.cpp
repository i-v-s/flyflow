#include "normdist.h"
#include <gtest/gtest.h>

bool NDEllipse::test(double x, double y) const
{
	x -= cx;
	y -= cy;
	double a = cos(alpha), b = sin(alpha);
	double x1 = a * x + b * y;
	double y1 = a * y - b * x;
	return x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry) < 1.0;
}

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

TEST(NormalDistributionTest, ellipse_test)
{
	NDEllipse e;
	e.cx = 1;
	e.cy = 2;
	e.rx = 1;
	e.ry = 2;
	e.alpha = -0.2;
	EXPECT_TRUE(e.test(1, 2));
	EXPECT_FALSE(e.test(1, 0));
	EXPECT_TRUE (e.test(0, 2));
	EXPECT_FALSE(e.test(0, 0));
	EXPECT_FALSE(e.test(0, 3));
}

const double PI = 3.1415926535897932384626433832795;

TEST(NormalDistributionTest, alpha)
{
	Eigen::Matrix2d s;
	Eigen::Vector2d m;
	m << 0, 0;
	s << 4.0, 0.0,
		 0.0, 1.0;
	NormalDistribution<double, 2> nd(m, s);
	EXPECT_NEAR(nd.ellipse(1).alpha, 0.0, 1E-6);
	EXPECT_NEAR(nd.ellipse(1).rx,    2.0, 1E-6);
	EXPECT_NEAR(nd.ellipse(1).ry,    1.0, 1E-6);
	nd.sigma << 
		1.0, 0.0,
		0.0, 2.0;
	EXPECT_NEAR(nd.ellipse(1).alpha, PI / 2, 1E-6);
	EXPECT_NEAR(nd.ellipse(1).rx, sqrt(2.0), 1E-6);
	EXPECT_NEAR(nd.ellipse(1).ry,       1.0, 1E-6);
	nd.sigma <<
		1.0, 0.0,
		0.0, 1.0;
	EXPECT_NEAR(nd.ellipse(1).alpha, 0.0, 1E-6);
	EXPECT_NEAR(nd.ellipse(1).rx,	 1.0, 1E-6);
	EXPECT_NEAR(nd.ellipse(1).ry,    1.0, 1E-6);
	nd.sigma << 
		1.0, 0.5,
		0.5, 1.0;
	EXPECT_NEAR(nd.ellipse(1).alpha, PI / 4, 1E-6);
	nd.sigma <<
		1.0,-0.5,
	   -0.5, 1.0;
	EXPECT_NEAR(nd.ellipse(1).alpha, -PI / 4, 1E-6);
}

void hitTest(double sx, double sy, double cv)
{
	Eigen::Matrix2d s;
	Eigen::Vector2d m;
	m << 0, 0;
	s << sx, cv,
		 cv, sy;
	NormalDistribution<double, 2> nd(m, s);
	NDEllipse e1 = nd.ellipse(1), e2 = nd.ellipse(2), e3 = nd.ellipse(3);
	int count = 100000;
	double s1 = 0, s2 = 0, s3 = 0;
	for (int x = 0; x < count; x++)
	{
		Eigen::Vector2d m;
		nd.generate(m);
		if (e1.test(m(0), m(1))) s1++;
		if (e2.test(m(0), m(1))) s2++;
		if (e3.test(m(0), m(1))) s3++;
	}
	EXPECT_NEAR(s1 / count, 0.3935, 0.0025);
	EXPECT_NEAR(s2 / count, 0.8647, 0.0025);
	EXPECT_NEAR(s3 / count, 0.9889, 0.0025);
}

TEST(NormalDistributionTest, hit_1x1)
{
	hitTest(1, 1, 0);
}

TEST(NormalDistributionTest, hit_10x1)
{
	hitTest(10, 1, 0);
}

TEST(NormalDistributionTest, hit_1x10)
{
	hitTest(1, 10, 0);
}

TEST(NormalDistributionTest, hit_1x10c3)
{
	hitTest(1, 10, 3);
}
