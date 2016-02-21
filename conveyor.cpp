#include "conveyor.h"

namespace flyflow {

Conveyor::Conveyor()
{

}

void Conveyor::onImage(const cv::Mat &image)
{
    cv::Mat mono;
    int t = image.type() & CV_MAT_DEPTH_MASK;
    if(t == CV_8U)
        image.copyTo(mono);
    else
        cv::cvtColor(image, mono, CV_BGR2GRAY);
    // TODO rectify

    if(jacobi_ == nullptr) jacobi_ = std::unique_ptr<Jacobi>(new Jacobi(mono.cols, mono.rows, 10));
    jacobi_->set(image);

    Frame nf(mono, 10);
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
    if(history_.size() > 5) history_.pop_back();
    history_.push_front(nf);
}


double Conveyor::solveLevel(const cv::Mat & f0, const cv::Mat & f1, cv::Mat & out)
{
    //std::vector<cv::Mat> v = {f0, f1, f1};
    //cv::merge(v, out);

    /*int h = f0.rows, w = f0.cols;
    cv::Mat whiteBox(f0.rows, f0.cols, CV_8U, cv::Scalar(100)), mask;
    double e = 1E10;
    cv::Mat optT = t_.clone();
    double step = 1.0;

    for(int x = 0; x < 20; x++)
    {
        cv::Mat f0t;
        cv::warpAffine(f0, f0t, t_, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(255));
        cv::warpAffine(whiteBox, mask, t_, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

        double te = calcError(f0t, f1, mask);
        cv::Mat du = calcAffineStep();


        if(te >= e)
        {
            t_ = optT.clone();
            step *= 0.3;
            if(step > 0.05) continue;
            if(e / e_.total() > 150)
            {
                cv::vconcat(out, cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 255)), out);
                return false;
            }
            return true;
        }
        std::vector<cv::Mat> v = {f0t, f1, f1};
        cv::merge(v, f0t);
        //if(!x) bt.copyTo(out);
        cv::vconcat(out, f0t, out);

        optT = t_.clone();
        e = te;
        t_ += du * step;
    }
    return true;*/
}

double Conveyor::solve(const Frame &f0, const Frame &f1, cv::Mat &pose)
{
    pose = cv::Mat::eye(2, 3, CV_64F);
    for(int l = 5; ; )
    {
        const cv::Mat & l0 = f0.levels_[l];
        const cv::Mat & l1 = f1.levels_[l];

        for(int it = 0; it < 10; it++)
        {
            cv::Mat l0t;
            //cv::warpAffine(l0, l0t, pose, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(255));
            //cv::warpAffine(whiteBox, mask, pose, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

        }
    }
    /*f0.levels_

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



/*double te = calcError(at, b, mask);
cv::Mat du = calcAffineStep();


if(te >= e)
{
    t_ = optT.clone();
    step *= 0.3;
    if(step > 0.05) continue;
    if(e / e_.total() > 150)
    {
        cv::vconcat(out, cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 255)), out);
        return false;
    }
    return true;
}
std::vector<cv::Mat> v = {at, b, b};
cv::merge(v, at);
//if(!x) bt.copyTo(out);
cv::vconcat(out, at, out);

optT = t_.clone();
e = te;
t_ += du * step;*/


}
