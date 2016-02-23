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

TEST(JacobiTest, Simple)
{
    Jacobi<int32_t, true, false, false, true, false, false> shiftJacobi;



}


}
