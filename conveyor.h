#ifndef CONVEYOR_H
#define CONVEYOR_H
#include <opencv2/opencv.hpp>
#include <deque>
#include "frame.h"
#include "jacobi.h"
#include <memory>

namespace flyflow {

class Conveyor
{
private:
    cv::Mat mask_;
    std::unique_ptr<Jacobi> jacobi_;
    std::deque<Frame> history_;
    double solve(const Frame &f0, const Frame &f1, cv::Mat & pose);
    double solveLevel(const cv::Mat &f0, const cv::Mat &b, cv::Mat &out);
public:
    Conveyor();
    void onImage(const cv::Mat & image);
};

}

#endif // CONVEYOR_H
