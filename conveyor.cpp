#include "conveyor.h"
#include <chrono>
#include <stdio.h>

#define SHOW_CALC

namespace flyflow {

Conveyor::Conveyor(Visualizer *vis): gn_(20), vis_(vis), pushHistory(true)
{

}

//Visualizer vt("test");

void Conveyor::onImage(const cv::Mat &mono)
{
    double min, max;
    cv::minMaxLoc(mono, &min, &max);
    if(min < min_) min_ = min;
    else min_ += (min - min_) * 0.03;
    if(max > max_) max_ = max;
    else max_ += (max - max_) * 0.03;

    Frame16 nf(mono, 10, min_, max_, invmed_);

    if(jacobi_.empty())
    {
        for(int l = nf.levels_.size(); l--; )
            jacobi_.push_back(JacobiAffine());
    }


    for(auto j = jacobi_.begin(); j != jacobi_.end(); j++)
    {
        if(j == jacobi_.begin()) j->set(mono, 128);
        else j->shrink(*(j - 1));
    }

    cv::Mat pose = cv::Mat::zeros(2, 3, CV_64F);
    double weight = 0;
    for(auto f = history_.begin(); f != history_.end(); f++)
    {
        cv::Mat t;
        double w = solve(*f, nf, t);
        weight += w;
        pose += t;
    }
    pose /= weight;
    if(pushHistory)
    {
        if(history_.size() > 0) history_.pop_back();
        history_.push_front(nf);
    }
}

std::string poseToStr(const cv::Mat & pose)
{
    std::string r;
    for(const double * p = (const double *) pose.data; p < (const double *)pose.dataend; p++)
    {
        if(r != "") r += ',';
        char buf[20];
        sprintf(buf, "%.3f", *p);
        r += buf;
    }
    return r;
}


double Conveyor::solve(const Frame16 &f0, const Frame16 &f1, cv::Mat &pose)
{
    //double min = std::min(f0.min_, f1.min_);
    //double max = std::max(f0.max_, f1.max_);
    double k = 1;//800 / (max - min);
    pose = cv::Mat::eye(2, 3, CV_64F);
    auto i0 = f0.levels_.rbegin(), i1 = f1.levels_.rbegin();
    double s = 1.0;// / (1 << (2 * jacobi_.size()));
    for(auto j = jacobi_.rbegin(); j != jacobi_.rend(); j++, i0++, i1++)
    {
        pose.at<double>(0, 2) *= 2;
        pose.at<double>(1, 2) *= 2;
        if(vis_)
        {
            cv::Mat v;
            cv::resize(*i1, v, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
            vis_->newColumn(v, s);
            vis_->add("p = " + poseToStr(pose), 'w');
            //vis_->add(*i0);
            //return 1.0;
        }
        s *= 4;
        if(vis_)
        {
            std::vector<cv::Mat> v;
            std::chrono::high_resolution_clock::time_point tp1 = std::chrono::high_resolution_clock::now();
            double e = gn_.solve<JacobiAffine, 1>(*i0, *i1, *j, pose, k, &v);
            std::chrono::high_resolution_clock::time_point tp2 = std::chrono::high_resolution_clock::now();

            //for(auto i : v) vis_->add(i);
            cv::Mat v1, v2;// = v.front(), & v2 = v.back();
            //cv::Mat V1
            cv::resize(v.front(), v1, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
            cv::resize(v.back(), v2, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
            vis_->add(v1);
            vis_->add(v2);
            vis_->add("e = " + std::to_string(e), 'w');
            std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(tp2 - tp1);
            vis_->add("t = " + std::to_string(dt.count()), 'w');
            vis_->add("k = " + std::to_string(k), 'w');
            std::stringstream ss;
            //ss << pose;
            vis_->add("p = " + poseToStr(pose), 'w');

        }
        else
            gn_.solve<JacobiAffine>(*i0, *i1, *j, pose, k);
        //solveLevel(*i0, *i1, *j, pose);
    }
}


}
