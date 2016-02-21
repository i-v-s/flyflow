#include "jacobi.h"
#include <gtest/gtest.h>

namespace flyflow
{

JacobiLevel::JacobiLevel(int w, int h, int level):
    gx_(h, w, cv_scalar), gy_(h, w, cv_scalar), h_(h), w_(w), ks_(1.0 / (32 << level)), ks2_(ks_ * ks_),
    j_{
         &gxx_, &gxy_, &gx_,
         &gyx_, &gyy_, &gy_
     }
{
    gxx_ = cv::Mat(h, w, cv_scalar);
    gxy_ = cv::Mat(h, w, cv_scalar);
    gyx_ = cv::Mat(h, w, cv_scalar);
    gyy_ = cv::Mat(h, w, cv_scalar);
}

TEST(JacobiTest, Simple)
{
    JacobiT<int32_t, true, false, false, true, false, false> shiftJacobi;



}

void JacobiLevel::set(const cv::Mat &f1)
{
    assert(h_ == f1.rows && w_ == f1.cols);
    cv::Mat gx, gy;
    cv::Sobel(f1, gx, CV_16S, 1, 0, CV_SCHARR);
    cv::Sobel(f1, gy, CV_16S, 0, 1, CV_SCHARR);
    gx.convertTo(gx_, cv_scalar);
    gy.convertTo(gy_, cv_scalar);

    mulByX<scalar>(gx_, gxx_);
    mulByY<scalar>(gx_, gxy_);
    mulByX<scalar>(gy_, gyx_);
    mulByY<scalar>(gy_, gyy_);
}

bool JacobiLevel::solve(const cv::Mat &error, cv::Mat &transf)
{
    int wm = 1, hm = 1;
    const double ks = 1.0 / 32;
    // Calculate vector B
    cv::Mat b = cv::Mat(6, 1, CV_64F);
    for(int y = 0; y < 6; y++)
        b.at<double>(y, 0) = scalMul<int32_t, int16_t>(*j_[y], error) * (ks / 100);

    // Calculate step
    cv::Mat dt;
    if(!cv::solve(a_, b, dt)) return false;
    dt.rows = 2;
    dt.cols = 3;
    dt.at<double>(0, 2) *= wm;
    dt.at<double>(1, 2) *= hm;
    return true;
}

void JacobiLevel::set(const JacobiLevel &src)
{
    shrink<scalar>(src.gx_, gx_);
    shrink<scalar>(src.gy_, gy_);

    mulByX<scalar>(gx_, gxx_);
    mulByY<scalar>(gx_, gxy_);
    mulByX<scalar>(gy_, gyx_);
    mulByY<scalar>(gy_, gyy_);
}

Jacobi::Jacobi(int w, int h, int min)
{
    int l = 0;
    while(w > min && h > min)
    {
        levels_.push_back(JacobiLevel(w, h, l++));
        w /= 2;
        h /= 2;
    }
}

void Jacobi::set(const cv::Mat &image)
{
    // TODO assert

    auto l = levels_.begin();
    l->set(image);
    for(l++; l != levels_.end(); l++)
        l->set(*(l - 1));
}

}
