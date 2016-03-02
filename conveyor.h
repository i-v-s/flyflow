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
    std::vector<Jacobi::Ptr> jacobi_;
    GaussNewton gn_;
    std::deque<Frame8> history_;
    double solve(const Frame8 &f0, const Frame8 &f1, cv::Mat & pose);
    Visualizer * vis_;
    double min_, max_;
    void initJacobi(int size);
public:
    cv::Mat invmed_;
    bool pushHistory;
    Conveyor(Visualizer * vis = 0);
    void onImage(const cv::Mat & mono);
};

}

#endif // CONVEYOR_H
