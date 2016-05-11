#include "kalman.h"
#include <gtest/gtest.h>
#include <cstdlib>

double rnd1(double min, double max)
{
	return ((double) rand() / RAND_MAX) * (max - min) + min;
}

TEST(KalmanTest, gauss_mul)
{
	srand(5);
	for (int x = 0; x < 100; x++)
	{
		double u1 = rnd1(-10, 10); // Матожидания
		double u2 = rnd1(-10, 10);
		double s1 = rnd1(0.01, 10); // Дисперсии
		double s2 = rnd1(0.01, 10);

		Kalman<double, 1, 1> k;
		EXPECT_DOUBLE_EQ(k.Sigma_(0, 0), 0.0);
		k.G_ << 1.0;
		k.Q_ << s1; // Шум за один шаг
		k.X_ << u1; // Исходная позиция
		k.predict();
		EXPECT_DOUBLE_EQ(k.X_(0), u1);
		EXPECT_DOUBLE_EQ(k.Sigma_(0, 0), s1);

		k.H_ << 1.0;
		k.R_ << s2; // Дисперсия датчика
		Eigen::Matrix<double, 1, 1> z;
		z << u2; // Датчик показал позицию
		k.correct(z);
		EXPECT_NEAR(k.Sigma_(0, 0),            s1 * s2 / (s1 + s2), 1E-10); // Произведение нормальных распределений
		EXPECT_NEAR(k.X_(0),       (u1 * s2 + u2 * s1) / (s1 + s2), 1E-10);
	}
}

TEST(KalmanTest, speed) // Сможет ли фильтр Кальмана измерить скорость?
{
	double dt = 0.1, v = 1.0; // Квант времени, скорость
	Kalman<double, 2, 1> k;
	k.X_ << 0.0, 0.0; // Положение, скорость
	k.G_(0, 1) = dt; // Задаём движение
	k.Q_(0, 0) = 0.1; // Нарастание неопределённости положения
	k.Q_(1, 1) = 0.1; // Нарастание неопределённости скорости
	k.Sigma_(1, 1) = 10; // Мы не знаем, какая скорость
	k.H_ << 1.0, 0.0; // Измеряем положение
	k.R_ << 0.1; // Шум измерения
	using namespace std;
	Eigen::Matrix<double, 1, 1> z;
	for (double t = 0; t < 10; t += dt)
	{
		z(0) = v * t;
		/*cout << endl << "t = " << t << endl;
		cout << "Z = " << z.transpose() << endl;
		cout << "X = " << k.X_.transpose() << endl;
		cout << "Sigma =" << endl << k.Sigma_ << endl;*/
		k.predict();
		k.correct(z);
	}
	EXPECT_NEAR(k.X_(1),    v, 0.0001);
	EXPECT_NEAR(k.X_(0), z(0), 0.0001);
}

TEST(KalmanTest, slam1d)
{
	Kalman<double, 4, 2> k;
	k.X_ << 0.0, 1.0, rnd1(0, 10), rnd1(0, 10); // Позиция, скорость, метка1, метка2
	double dt = 0.1;
	k.G_(1, 0) = dt; // позиция = позиция + скорость * dt
	k.Sigma_(0, 0) = 0.1;
	k.Sigma_(1, 1) = 0.1;
	k.Sigma_(2, 2) = 100; // Мы не знаем, где метка
	k.Sigma_(3, 3) = 100; // Мы не знаем, где метка
	k.Q_(0, 0) = 1; // 
	k.Q_(1, 1) = 1; // 
	k.Q_(2, 2) = 0; // Метка не двигается

	k.H_ << -1.0, 0.0, 1.0, 0.0,
			-1.0, 0.0, 0.0, 1.0;
	k.R_(0, 0) = 0.1;
	k.R_(1, 1) = 0.1;

	Eigen::Matrix<double, 2, 1> z;
	double mark1 = 10;// rnd(0, 10); // Реальная позиция метки
	double mark2 = 20;// rnd(0, 10); // Реальная позиция метки
	
	for (double t = 0; t < 1; t += dt)
	{
		z(0) = mark1 - k.X_(0);
		z(1) = mark2 - k.X_(1);
		//std::cout << "X =" << std::endl << k.X_ << std::endl;
		//std::cout << "Sigma =" << std::endl << k.Sigma_ << std::endl;
		k.predict();
		k.correct(z);

	}

}


