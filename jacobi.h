#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace flyflow
{

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
        //std::cout << "cv_type<Te>() = " << cv_type<Te>() << "; e.type() = " << e.type() << std::endl;
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
            if(gx_.empty()) gx_ = cv::Mat(p.gx_.rows / 2, p.gx_.cols / 2, cv_type<T>());
            flyflow::shrink<T>(p.gx_, gx_);
        }
        if(usegy)
        {
            if(gy_.empty()) gy_ = cv::Mat(p.gy_.rows / 2, p.gy_.cols / 2, cv_type<T>());
            flyflow::shrink<T>(p.gy_, gy_);
        }
        calcA();
    }
    template<class Te> inline void solve(const cv::Mat & e, Vector & r) const
    {
        Vector b;
        calcB<Te>(b, e);
        r = invA_ * b;
    }
    template<class Te> void solve(const cv::Mat & e, cv::Mat & dt) const
    {
        assert(dt.rows == 2 && dt.cols == 3 && dt.type() == CV_64F);
        Vector v;
        solve<Te>(e, v);
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

class GaussNewton
{
private:
    int maxStepCount_;
public:
    GaussNewton(int maxStepCount) : maxStepCount_(maxStepCount) {}
    template<class J, class V> double solve(const cv::Mat & f0, const cv::Mat & f1, const J & j, cv::Mat & pose, V * vis = nullptr)
    {
        //std::vector<cv::Mat> v = {f0, f1, f1};
        //cv::merge(v, out);

        cv::Mat du(2, 3, CV_64F);
        int h = f0.rows, w = f0.cols;
        cv::Mat whiteBox(f0.rows, f0.cols, CV_8U, cv::Scalar(100)), mask;
        double e = 1E10;
        cv::Mat p = pose.clone();
        double step = 1.0;

        for(int x = 0; x < 20; x++)
        {
            cv::Mat f0t;
            cv::warpAffine(f0, f0t, p, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(255));
            cv::warpAffine(whiteBox, mask, p, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

            cv::Mat em;
            cv::subtract(f1, f0t, em, mask, CV_16S);
            double te = cv::norm(em);
            j.solve<int16_t>(em, du);


            if(te >= e)
            {
                p = pose.clone();
                step *= 0.3;
                if(step > 0.05) continue;
                if(e / em.total() > 150)
                {
                    //cv::vconcat(out, cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 255)), out);
                    if(vis) vis->add("error");
                    return false;
                }
                return true;
            }
            std::vector<cv::Mat> v = {f0t, f1, f1};
            if(vis)
            {
                vis->add(f0t);
            }
            //cv::merge(v, f0t);
            //cv::vconcat(out, f0t, out);

            pose = p.clone();
            e = te;
            p += du * step;
        }
        return true;
    }
};


}
#endif // FRAME_H
