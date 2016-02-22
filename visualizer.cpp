#include "visualizer.h"

namespace flyflow
{

Visualizer::Visualizer(std::string name): name_(name)
{
    type_ = CV_8UC3;
}

void Visualizer::add(const cv::Mat &image)
{
    cv::Mat scaled;
    image.convertTo(scaled, CV_8U, scale_);
    std::vector<cv::Mat> v = {ref_, scaled, scaled};
    cv::Mat im;
    cv::merge(v, im);
    if(column_.empty()) column_ = im;
    else
    {
        assert(column_.cols == im.cols);
        cv::vconcat(column_, im, column_);
    }
}

void Visualizer::newColumn(const cv::Mat & ref, double scale)
{
    ref.convertTo(ref_, CV_8U, scale);
    cv::Mat col = column_;
    scale_ = scale;
    if(result_.empty()) result_ = col;
    else
    {
        if(col.rows > result_.rows) cv::vconcat(result_, cv::Mat(col.rows - result_.rows, result_.cols, type_, cv::Scalar(0, 0, 0)), result_);
        else if(result_.rows > col.rows) cv::vconcat(col, cv::Mat(result_.rows - col.rows, col.cols, type_, cv::Scalar(0, 0, 0)), col);
        cv::hconcat(result_, col, result_);
    }
    column_ = cv::Mat();
}

void Visualizer::show()
{
    if(result_.empty()) result_ = column_;

    if(!result_.empty()) cv::imshow(name_, result_);
    result_ = cv::Mat();
    column_ = cv::Mat();
}

void Visualizer::add(const std::string &text, char c)
{
    cv::Scalar s(0, 0, 0);
    switch(c)
    {
    case 'r': s = cv::Scalar(0, 0, 255); break;
    case 'g': s = cv::Scalar(0, 255, 0); break;
    case 'b': s = cv::Scalar(255, 0, 0); break;
    }

    cv::putText(column_, text, cv::Point(5,5), cv::FONT_HERSHEY_SIMPLEX, 5, s);
}

}
