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
    std::deque<Frame16> history_;
    double solve(const Frame16 &f0, const Frame16 &f1, cv::Mat & pose);
    Visualizer * vis_;
    double min_, max_;
public:
    cv::Mat invmed_;
    bool pushHistory;
    Conveyor(Visualizer * vis = 0);
    void onImage(const cv::Mat & mono);
};

}

#endif // CONVEYOR_H
