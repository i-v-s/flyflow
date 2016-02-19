#ifndef IMAGEFRAME_H
#define IMAGEFRAME_H
#include <opencv2/opencv.hpp>
#include <vector>

namespace flyflow
{

class Frame
{
public:
    std::vector<cv::Mat> levels_;
    cv::Mat pose_;
    Frame(const cv::Mat & image, int min);
};

}
#endif // IMAGEFRAME_H
