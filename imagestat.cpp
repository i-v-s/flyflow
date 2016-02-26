#include "imagestat.h"

void ImageStat::onImage(const cv::Mat &image)
{
    if(sum_.empty())
        image.convertTo(sum_, CV_32F);
    else
        cv::add(sum_, image, sum_, cv::noArray(), CV_32F);
        //sum_ += image;
    if(sum2_.empty())
    {
        image.convertTo(sum2_, CV_32F);
        cv::multiply(sum2_, sum2_, sum2_, 1, CV_32F);
    }
    else
    {
        cv::Mat t;
        cv::multiply(image, image, t, 1, CV_32F);
        sum2_ += t;
    }

    count_++;
    if(show_)
    {
        autoShow(sum_, "median");
        cv::Mat t;
        cv::multiply(sum_, sum_, t, 1.0 / (count_), CV_32F);
        cv::subtract(sum2_, t, t);
        autoShow(t, "disperse");
    }
}

void ImageStat::autoShow(const cv::Mat &image, const std::string &name)
{
    double min, max;
    cv::minMaxLoc(image, &min, &max);
    if(min == max) return;
    double a = 255 / (max - min), b = -min * a;
    cv::Mat im;
    //sum_.convertTo(im, CV_8U, 1.0 / count_);
    image.convertTo(im, CV_8U, a, b);
    cv::imshow(name, im);
}

void ImageStat::reset()
{
    count_ = 0;
    sum_ = cv::Mat();
    sum2_ = cv::Mat();
}

void ImageStat::save()
{
    cv::Mat im;
    sum_.convertTo(im, CV_8U, 1.0 / count_);
    cv::imwrite("median.png", im);
}

ImageStat::ImageStat(bool show): count_(0), show_(show)
{

}
