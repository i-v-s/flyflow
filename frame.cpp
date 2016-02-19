#include "frame.h"
#include "utils.h"

namespace flyflow {


Frame::Frame(const cv::Mat & image, int min)
{
    int w = image.cols, h = image.rows;
    while(w > min && h > min)
    {
        cv::Mat i;
        if(levels_.empty())
            image.convertTo(i, cv_scalar);
        else
        {
            i = cv::Mat(h, w, cv_scalar);
            shrink<scalar>(levels_.back(), i);
        }
        levels_.push_back(i);
        w /= 2;
        h /= 2;
    }
}


}
