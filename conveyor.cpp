#include "conveyor.h"
#include <chrono>
#include <stdio.h>
#include "timer.h"

#define SHOW_CALC

namespace flyflow {

Conveyor::Conveyor(Visualizer *vis): gn_(0.2, 10), vis_(vis), pushHistory(true)
{

}

//Visualizer vt("test");

cv::Mat mulPose(const cv::Mat & p1, const cv::Mat & p2)
{
    cv::Mat v1 = p1(cv::Rect(0, 0, 2, 2));

}

void Conveyor::onImage(const cv::Mat &mono)
{    
    Timer timer;
    timer.start();
    double min, max;
    cv::minMaxLoc(mono, &min, &max);
    //if(min < min_) min_ = min;
    min_ += (min - min_) * 0.02;
    //if(max > max_) max_ = max;
    max_ += (max - max_) * 0.02;

    Frame8 nf(mono, 10, min_, max_, invmed_);

    if(jacobi_.empty()) initJacobi(nf.levels_.size());

    for(auto j = jacobi_.begin(); j != jacobi_.end(); j++)
    {
        if(j == jacobi_.begin()) (*j)->set(nf.levels_[0], 1);
        else (*j)->shrink((j - 1)->get());
    }

    cv::Mat pose = cv::Mat::zeros(2, 3, CV_64F);
    double weight = 0;
    for(auto f = history_.begin(); f != history_.end(); f++)
    {
        cv::Mat t;
        double w = solve(*f, nf, t);

        /*cv::Mat tr = f->pose_(cv::Rect(0, 0, 2, 2));
        cv::Mat to = t(cv::Rect(2, 0, 1, 2));
        ps += tr * t(cv::Rect(2, 0, 1, 2));
        tr *= t(cv::Rect(0, 0, 2, 2));

        t =*/
        weight += w;
        pose += t;
    }
    pose /= weight;
    nf.pose_ = pose;
    if(pushHistory)
    {
        if(history_.size() > 0) history_.pop_back();
        history_.push_front(nf);
    }
    double time = timer.end();
    std::cout << "time = " << time << std::endl;
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


double Conveyor::solve(const Frame8 &f0, const Frame8 &f1, cv::Mat &pose)
{
    //double min = std::min(f0.min_, f1.min_);
    //double max = std::max(f0.max_, f1.max_);
    //double k = 1;//800 / (max - min);
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
            double e = gn_.solve<1>(*i0, *i1, j->get(), pose, &v);
            std::chrono::high_resolution_clock::time_point tp2 = std::chrono::high_resolution_clock::now();

            //for(auto i : v) vis_->add(i);
            cv::Mat v1, v2, e2;// = v.front(), & v2 = v.back();
            //cv::Mat V1
            cv::resize(v.front(), v1, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
            cv::resize(*(v.rbegin() + 1), v2, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
            cv::resize(v.back(), e2, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
            vis_->add(v1);
            vis_->add(v2);
            vis_->add(e2);
            vis_->add("e = " + std::to_string(e), 'w');
            std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(tp2 - tp1);
            vis_->add("t = " + std::to_string(dt.count()), 'w');
            //vis_->add("k = " + std::to_string(k), 'w');
            std::stringstream ss;
            //ss << pose;
            vis_->add("steps = " + std::to_string(gn_.stepCount_), 'w');
            vis_->add("p = " + poseToStr(pose), 'w');

        }
        else
            gn_.solve(*i0, *i1, j->get(), pose);
        //solveLevel(*i0, *i1, *j, pose);
    }
}

void Conveyor::initJacobi(int size)
{
    for(int l = size; l--; )
    {
        Jacobi::Type t = (l > 0) ? Jacobi::jtAffine : Jacobi::jtShift;
        jacobi_.push_back(Jacobi::create(t));
    }
}


}
