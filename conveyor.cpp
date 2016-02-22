#include "conveyor.h"

#define SHOW_CALC

namespace flyflow {

Conveyor::Conveyor(Visualizer *vis): gn_(20), vis_(vis)
{

}

void Conveyor::onImage(const cv::Mat &image)
{
    cv::Mat mono;
    if(image.type() == CV_8U)
        image.copyTo(mono);
    else
        cv::cvtColor(image, mono, CV_RGB2GRAY);
    // TODO rectify

    Frame32 nf(mono, 40);
    ;
    if(jacobi_.empty())
    {
        double scale = 1.0;
        for(int l = nf.levels_.size(); l--; scale /= 4)
            jacobi_.push_back(JacobiAffine(scale));
    }


    for(auto j = jacobi_.begin(); j != jacobi_.end(); j++)
    {
        if(j == jacobi_.begin()) j->set(mono);
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
    if(history_.size() > 0) history_.pop_back();
    history_.push_front(nf);
}

double Conveyor::solve(const Frame32 &f0, const Frame32 &f1, cv::Mat &pose)
{
    pose = cv::Mat::eye(2, 3, CV_64F);
    auto i0 = f0.levels_.rbegin(), i1 = f1.levels_.rbegin();
    double s = 1.0;// / (1 << (2 * jacobi_.size()));
    for(auto j = jacobi_.rbegin(); j != jacobi_.rend(); j++, i0++, i1++)
    {
        pose.at<double>(0, 2) *= 2;
        pose.at<double>(1, 2) *= 2;
        if(vis_)
        {
            vis_->newColumn(*i1, s);
            vis_->add(*i0);
            return 1.0;
        }
        s *= 4;
        double e = gn_.solve<JacobiAffine, Visualizer>(*i0, *i1, *j, pose, vis_);
        //solveLevel(*i0, *i1, *j, pose);
    }
    /*
    while(gnn.demo(prev->pyramid[lvl], f->pyramid[lvl], out) && lvl > 1)
    {
        if(all.empty()) all = out;
        else
        {
            if(out.rows > all.rows) cv::vconcat(all, cv::Mat(out.rows - all.rows, all.cols, CV_8UC3, cv::Scalar(0, 0, 0)), all);
            else if(all.rows > out.rows) cv::vconcat(out, cv::Mat(all.rows - out.rows, out.cols, CV_8UC3, cv::Scalar(0, 0, 0)), out);
            cv::hconcat(all, out, all);
        }
        cout << "Te" << lvl << " = " << endl << gnn.t_ << endl << endl;
        lvl--;
        gnn.level_ = lvl;
        gnn.scaleBy(2);
        cout << "Tb" << lvl << " = " << endl << gnn.t_ << endl << endl;
    }
    if(all.empty()) all = out;
    cv::resize(all, all, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
    cv::imshow("all", all);

}*/

}


}
