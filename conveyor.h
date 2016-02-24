#ifndef CONVEYOR_H
#define CONVEYOR_H
#include <opencv2/opencv.hpp>
#include <deque>
#include "frame.h"
#include "jacobi.h"
#include <memory>
#include "visualizer.h"

namespace flyflow {

class Conveyor
{
private:
    cv::Mat mask_;
    std::vector<JacobiAffine> jacobi_;
    GaussNewton gn_;
    std::deque<Frame32> history_;
    double solve(const Frame32 &f0, const Frame32 &f1, cv::Mat & pose);
    Visualizer * vis_;
public:
    bool pushHistory;
    Conveyor(Visualizer * vis = 0);
    void onImage(const cv::Mat & image);
};

}

#endif // CONVEYOR_H
