#ifndef IMAGEFRAME_H
#define IMAGEFRAME_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "utils.h"

namespace flyflow
{

template<class Ts, class Td>
void distort(const cv::Mat & src, cv::Mat & dst, double min, double max)
{
    const Ts * sp = (const Ts *) src.data, * e = src.dataend;
    Td * dp = (Td *) dst.data;
    for( ; sp < e; sp++, dp++)
    {
        double s = (*sp - min) / (max - min);
        *dp = (1.5708 + atan((s - 0.5) * 1.5)) * 10000;
    }
}

template<class T> class Frame
{
public:
    std::vector<cv::Mat> levels_;
    cv::Mat pose_;
    //double min_, max_;
    Frame(const cv::Mat & image, int min, double min_, double max_, const cv::Mat & invmed = cv::Mat())
    {
        cv::minMaxLoc(image, &min_, &max_);
        int w = image.cols, h = image.rows;
        while(w > min && h > min)
        {
            cv::Mat i;
            if(levels_.empty())
            {
                /*if(!invmed.empty())
                    cv::addWeighted(image, 128, invmed, 128, 0, i, cv_type<T>());
                else
                    image.convertTo(i, cv_type<T>(), 128);*/
                i = cv::Mat(h, w, cv_type<T>());
                distort<uint8_t, T>(image, i, min_, max_);
            }
            else
            {
                i = cv::Mat(h, w, cv_type<T>());
                shrink<T>(levels_.back(), i);
            }
            levels_.push_back(i);
            w /= 2;
            h /= 2;
        }
    }
};

typedef Frame<uint16_t> Frame16;

}
#endif // IMAGEFRAME_H
