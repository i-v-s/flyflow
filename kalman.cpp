#include "kalman.h"
#include <gtest/gtest.h>
#include <cstdlib>

// СВ, равномерно от -1 до 1
double rnd()
{
	return ((double) rand() / RAND_MAX) * 2.0 - 1.0;
}

//Подпрограмма нормального распределения - возв СВ по гауссу с МО 0 и дисп 1.                
double rand_norm()
{
	static double r;
	static bool flag = false;
	if(flag) { flag = false; return(r);	}
	while(1)
	{
		double v1 = rnd(), v2 = rnd(), s = v1 * v1 + v2 * v2;
		if(s <= 1.0 && s > 0.0)
		{
			r = v1 * sqrt(-2.0 * log(s) / s); //за один раз получаем 2 СВ
			flag = 1;
			return v2 * sqrt(-2.0 * log(s) / s);
		}
	}
}

TEST(KalmanTest, gauss_mul)
{
	srand(5);
	for (int x = 0; x < 100; x++)
	{
		double u1 = ((double)rand() / RAND_MAX) * 20.0 - 10.0; // Матожидания
		double u2 = ((double)rand() / RAND_MAX) * 20.0 - 10.0;
		double s1 = ((double)rand() / RAND_MAX) * 5.0 + 0.1; // Дисперсии
		double s2 = ((double)rand() / RAND_MAX) * 5.0 + 0.1;

		Kalman<double, 1> k;
		EXPECT_DOUBLE_EQ(k.Sigma_(0, 0), 0.0);
		k.G_ << 1.0;
		k.Q_ << s1; // Шум за один шаг
		k.X_ << u1; // Исходная позиция
		k.predict();
		EXPECT_DOUBLE_EQ(k.X_(0), u1);
		EXPECT_DOUBLE_EQ(k.Sigma_(0, 0), s1);

		Corrector<double, 1, 1> c;
		c.H_ << 1.0;
		c.R_ << s2; // Дисперсия датчика
		Eigen::Matrix<double, 1, 1> z;
		z << u2; // Датчик показал позицию
		c.correct(k.X_, k.Sigma_, z);
		EXPECT_NEAR(k.Sigma_(0, 0),            s1 * s2 / (s1 + s2), 1E-10); // Произведение нормальных распределений
		EXPECT_NEAR(k.X_(0),       (u1 * s2 + u2 * s1) / (s1 + s2), 1E-10);
	}
}


