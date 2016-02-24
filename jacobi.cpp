#include "jacobi.h"
#include <gtest/gtest.h>
#include <iostream>
#include <random>

namespace flyflow
{

TEST(JacobiTest, Gradient)
{
    Jacobi<int32_t, false, false, true, true, false, true> j;
    int w = 3, h = 3;
    cv::Mat a(h, w, CV_8U);
    for(int t = 0; t < 100; t++)
    {
        uint8_t * p = a.data;
        int mx = std::rand() % 64;
        int my = std::rand() % 64;
        for(int y = 0; y < h; y++) for(int x = 0; x < w; x++)
            *(p++) = x * mx + y * my;
        j.set(a);
        //std::cout << "gx = " << std::endl << j.gx() << std::endl << std::endl;
        EXPECT_NEAR(j.gx().at<int32_t>(1, 1) * j.scale(), mx, 1E-6);
        EXPECT_NEAR(j.gy().at<int32_t>(1, 1) * j.scale(), my, 1E-6);
    }
}

TEST(JacobiTest, ShrinkedGradient)
{
    Jacobi<int32_t, false, false, true, true, false, true> j0, j1, j2;
    int w = 3, h = 3;
    cv::Mat a(h, w, CV_8U);
    for(int t = 0; t < 100; t++)
    {
        uint8_t * p = a.data;
        int mx = std::rand() % 64;
        int my = std::rand() % 64;
        for(int y = 0; y < h; y++) for(int x = 0; x < w; x++)
            *(p++) = x * mx + y * my;
        j.set(a);
        //std::cout << "gx = " << std::endl << j.gx() << std::endl << std::endl;
        EXPECT_NEAR(j.gx().at<int32_t>(1, 1) * j.scale(), mx, 1E-6);
        EXPECT_NEAR(j.gy().at<int32_t>(1, 1) * j.scale(), my, 1E-6);
    }
}

TEST(JacobiTest, Shift)
{
    Jacobi<int32_t, false, false, true, false, false, true> sj;
    int w = 20, h = 20;
    cv::Mat a(h, w, CV_8U), b;
    uint8_t * p = a.data;
    for(int y = 0; y < h; y++) for(int x = 0; x < w; x++)
        *(p++) = x * (w - x) + 2 * y;
    sj.set(a);
    cv::Mat t = cv::Mat::eye(2, 3, CV_64F);
    cv::Mat du = cv::Mat(2, 3, CV_64F);
    cv::Mat wb(h, w, CV_8U, cv::Scalar(255)), m;
    t.at<double>(0, 2) = 2;

    for(int x = 0; x < 10; x++)
    {

        cv::warpAffine(wb, m, t, cv::Size(h, w));
        cv::warpAffine(a, b, t, cv::Size(h, w));
        cv::Mat e;
        cv::subtract(b, a, e, m, CV_16S);
        sj.solve<int16_t>(e, du);
        std::cout << "!!! iteration " << x + 1 << std::endl;
        //cout << "gx = " << endl << gx_ << endl << endl;
        //cout << "gy = " << endl << gy_ << endl << endl;
        //cout << "t = " << endl << t << endl << endl;
        //cout << "a = " << endl << a_ << endl << endl;
        //cout << "b = " << endl << b_ << endl << endl;
        std::cout << "du = " << std::endl << du << std::endl << std::endl;
        //t.at<double>(0, 2) += du.at<double>(0, 0);
        //t.at<double>(1, 2) += du.at<double>(1, 0);
    }



}


}
