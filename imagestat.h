#ifndef IMAGESTAT_H
#define IMAGESTAT_H
#include <opencv2/opencv.hpp>

class ImageStat
{
private:
    cv::Mat sum_, sum2_;
    int count_;
    bool show_;
public:
    void onImage(const cv::Mat & image);
    void autoShow(const cv::Mat & image, const std::string & name);
    void reset();
    void save();
    ImageStat(bool show = true);
};

#endif // IMAGESTAT_H
