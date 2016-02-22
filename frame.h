#ifndef IMAGEFRAME_H
#define IMAGEFRAME_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "utils.h"

namespace flyflow
{

template<class T> class Frame
{
public:
    std::vector<cv::Mat> levels_;
    cv::Mat pose_;
    Frame(const cv::Mat & image, int min)
    {
        int w = image.cols, h = image.rows;
        while(w > min && h > min)
        {
            cv::Mat i;
            if(levels_.empty())
                image.convertTo(i, cv_type<T>());
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

typedef Frame<uint16_t> Frame32;

}
#endif // IMAGEFRAME_H
