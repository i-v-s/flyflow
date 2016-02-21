#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "utils.h"

namespace flyflow
{

template<class T> int cv_type();
template<> inline int cv_type<uint16_t>() {return CV_16U;}
template<> inline int cv_type<int16_t>() {return CV_16S;}
template<> inline int cv_type<int32_t>() {return CV_32S;}


template<class T, bool usegx = false, bool usegxx = false, bool usegxy = false,
                  bool usegy = false, bool usegyx = false, bool usegyy = false>
class JacobiT
{
public:
    cv::Mat gx_, gy_; // Gradients
    cv::Mat j_, a_;
    inline int size(){ return (usegx?1:0) + (usegxx?1:0) + (usegxy?1:0) + (usegy?1:0) + (usegyx?1:0) + (usegyy?1:0);}
    typedef double (* Multiplier) (const cv::Mat & m1, const cv::Mat & m2);

    std::vector<Multiplier> muls_;
    JacobiT()
    {
        /*j_{
             &gxx_, &gxy_, &gx_,
             &gyx_, &gyy_, &gy_
         }*/
        if(usegxx)
        {
            muls_.push_back(&gradMul<T, T, muxXX, >);
        }
        for(int x = 0; x < 6; x++)
        {

            Multiplier mul = &gradMul<T, T, muxXX, false, false>;
            muls_.push_back(mul);
        }
    }
    void set(const cv::Mat &f1)
    {
        //assert(h_ == f1.rows && w_ == f1.cols);
        cv::Mat gx, gy;
        if(usegx)
        {
            cv::Sobel(f1, gx, CV_16S, 1, 0, CV_SCHARR);
            if(cv_type<T>() == CV_16S) gx_ = gx;
            else gx.convertTo(gx_, cv_type<T>());
        }
        if(usegy)
        {
            cv::Sobel(f1, gy, CV_16S, 0, 1, CV_SCHARR);
            if(cv_type<T>() == CV_16S) gy_ = gy;
            else gy.convertTo(gy_, cv_type<T>());
        }
        a_ = cv::Mat(size(), size(), CV_64F);
        auto m = muls_.begin();
        for(int x = 0; x < size(); x++)
        {
            a_.at<double>(x, x) = (*(m++))(gx_, gy_);
            for(int y = 0; y < x; y++)
            {
                double t = (*(m++))(gx_, gy_);
                a_.at<double>(x, y) = t;
                a_.at<double>(y, x) = t;
            }
        }
    }
};

//typedef Jacobi

class JacobiLevel
{
private:
    int w_, h_;
    const float ks_, ks2_;
    cv::Mat f; // Image
    cv::Mat gx_, gy_; // Gradients
    cv::Mat gxx_, gyy_, gxy_, gyx_;
    cv::Mat a_;
    cv::Mat * j_[6];
public:
    JacobiLevel(int w, int h, int level);
    void set(const JacobiLevel & src);
    void set(const cv::Mat & f1);
    bool solve(const cv::Mat & error, cv::Mat & transf);
};

class Jacobi
{
private:
    std::vector<JacobiLevel> levels_;
public:
    Jacobi(int w, int h, int min = 10);
    void set(const cv::Mat & image);
};

}
#endif // FRAME_H
