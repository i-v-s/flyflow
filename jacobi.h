#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace flyflow
{

template<class T> constexpr int cv_type();
template<> constexpr int cv_type<uint16_t>() {return CV_16U;}
template<> constexpr int cv_type<int16_t>() {return CV_16S;}
template<> constexpr int cv_type<int32_t>() {return CV_32S;}


template<class T, bool usegxx = false, bool usegxy = false, bool usegx = true,
                  bool usegyx = false, bool usegyy = false, bool usegy = true>
class Jacobi
{
public:
    static constexpr int size() { return (usegx?1:0) + (usegxx?1:0) + (usegxy?1:0) + (usegy?1:0) + (usegyx?1:0) + (usegyy?1:0);}
    typedef Eigen::Matrix<double, size(), size()> Matrix;
    typedef Eigen::Matrix<double, size(), 1> Vector;
private:
    const double scale_;
    cv::Mat gx_, gy_; // Gradients
    Eigen::Matrix<double, size(), size()> invA_;
    void calcA()
    {
        Matrix a = Matrix::Zero();
        const T * p1, * p2;
        double gx, gy;
        Vector v;
        if(usegx) p1 = (const T *) gx_.data;
        if(usegy) p2 = (const T *) gy_.data;
        int w = usegx ? gx_.cols : gy_.cols;
        int h = usegx ? gx_.rows : gy_.rows;
        for(int y = 0; y < h; y++)
            for(int x = 0; x < w; x++)
            {
                double * r = v.data();
                if(usegx) gx = *(p1++);
                if(usegy) gy = *(p2++);
                if(usegxx) *(r++) = gx * x;
                if(usegxy) *(r++) = gx * y;
                if(usegx) *(r++) = gx;

                if(usegyx) *(r++) = gy * x;
                if(usegyy) *(r++) = gy * y;
                if(usegy) *(r++) = gy;
                assert(r - v.data() == size());
                a += v * v.transpose();
            }
        a *= scale_ * scale_;
        invA_ = a.inverse();
    }
    template<class Te> void calcB(Vector & b, const cv::Mat & e) const
    {
        assert(!usegx || (e.rows == gx_.rows && e.cols == gx_.cols));
        assert(!usegy || (e.rows == gy_.rows && e.cols == gy_.cols));
        assert(cv_type<Te>() == e.type());
        b.setZero();
        const T * p1, * p2;
        double gx, gy;
        Vector v;
        if(usegx) p1 = (const T *) gx_.data;
        if(usegy) p2 = (const T *) gy_.data;
        const Te * pe = (const Te *) e.data;
        int w = e.cols;
        int h = e.rows;
        for(int y = 0; y < h; y++)
            for(int x = 0; x < w; x++)
            {
                double * r = v.data();
                if(usegx) gx = *(p1++);
                if(usegy) gy = *(p2++);
                if(usegxx) *(r++) = gx * x;
                if(usegxy) *(r++) = gx * y;
                if(usegx) *(r++) = gx;

                if(usegyx) *(r++) = gy * x;
                if(usegyy) *(r++) = gy * y;
                if(usegy) *(r++) = gy;
                assert(r - v.data() == size());
                b += v * *(pe++);
            }
        b *= scale_;
    }

public:
    Jacobi(double scale = 1.0): scale_(scale / 32) {}
    void set(const cv::Mat &f)
    {
        //assert(h_ == f1.rows && w_ == f1.cols);
        cv::Mat gx, gy;
        if(usegx)
        {
            cv::Sobel(f, gx, CV_16S, 1, 0, CV_SCHARR);
            if(cv_type<T>() == CV_16S) gx_ = gx;
            else gx.convertTo(gx_, cv_type<T>());
        }
        if(usegy)
        {
            cv::Sobel(f, gy, CV_16S, 0, 1, CV_SCHARR);
            if(cv_type<T>() == CV_16S) gy_ = gy;
            else gy.convertTo(gy_, cv_type<T>());
        }
        calcA();
    }
    void shrink(const Jacobi & p)
    {
        if(usegx)
        {
            if(gx_.empty()) gx_ = cv::Mat(p.gx_.cols, p.gx_.rows, cv_type<T>());
            flyflow::shrink<T>(p.gx_, gx_);
        }
        if(usegy)
        {
            if(gy_.empty()) gy_ = cv::Mat(p.gy_.cols, p.gy_.rows, cv_type<T>());
            flyflow::shrink<T>(p.gy_, gy_);
        }
        calcA();
    }
    template<class Te> inline void solve(const cv::Mat & e, Vector & r) const
    {
        Vector b;
        calcB<Te>(b, e);
        r = b * invA_;
    }
    template<class Te> void solve(const cv::Mat & e, cv::Mat & dt) const
    {
        assert(dt.rows == 2 && dt.cols == 3 && dt.type() == CV_64F);
        Vector v;
        solve(e, v);
        const double * pv = v.data();
        double * pd = (double *) dt.data;
        *(pd++) = usegxx ? *(pv++) : 0;
        *(pd++) = usegxy ? *(pv++) : 0;
        *(pd++) = usegx ? *(pv++) : 0;
        *(pd++) = usegyx ? *(pv++) : 0;
        *(pd++) = usegyy ? *(pv++) : 0;
        *(pd++) = usegy ? *(pv++) : 0;
    }
};

typedef Jacobi<int32_t> JacobiShift;
typedef Jacobi<int32_t, true, true, true, false, false, false> JacobiStereoX;
typedef Jacobi<int32_t, true, true, true, true, true, true> JacobiAffine;


}
#endif // FRAME_H
