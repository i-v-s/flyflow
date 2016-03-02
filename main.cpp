#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

#include "conveyor.h"
#include "visualizer.h"
#include "imagestat.h"

using namespace std;
using namespace  flyflow;

Visualizer visgn("GN");
Conveyor conv;//&visgn);
ImageStat is;
int contrast = 0, brightness = 50, stepMul = 0;


bool useStat = false;


void showPos()
{
    /*cv::Point pt, cpt(320, 240);

    ps += tr * gnn.t_(cv::Rect(2, 0, 1, 2));
    tr *= gnn.t_(cv::Rect(0, 0, 2, 2));

    pt.x = (int)ps.at<double>(0, 0) * 0.2;
    pt.y = (int)ps.at<double>(1, 0) * 0.2;
    cv::line(traj, oldPt + cpt, pt + cpt, cv::Scalar(255, 0, 0));
    cv::Mat view = traj.clone();

    cv::circle(view, trans(tr, 5, 5) + pt + cpt, 4, cv::Scalar(0, 0, 255));
    cv::circle(view, trans(tr, -5, 5) + pt + cpt, 4, cv::Scalar(0, 0, 255));
    cv::circle(view, trans(tr, 5, -5) + pt + cpt, 4, cv::Scalar(0, 0, 0));
    cv::circle(view, trans(tr, -5, -5) + pt + cpt, 4, cv::Scalar(0, 0, 0));*/
}

bool onImage(const cv::Mat & image)
{
    //cv::imshow("image", image);

    cv::Mat mono;
    static cv::Mat paused;
    if(!paused.empty())
        mono = paused;
    else if(image.type() == CV_8U)
        image.copyTo(mono);
    else
        cv::cvtColor(image, mono, CV_RGB2GRAY);
    // TODO rectify
    cv::Mat mono1;
    mono.convertTo(mono1, CV_8U, exp(contrast * 0.1), (brightness - 50) * 25.0);

    //cv::imshow("cam", frame);
    if(useStat)is.onImage(mono);
    conv.onImage(mono1);


    visgn.show();
    char k = cv::waitKey(1);
    switch(k)
    {
    case 'i':
        conv.pushHistory = !conv.pushHistory;
        break;
    case 'm':
        useStat = true;
        is.reset();
        break;
    case 'p':
        if(paused.empty()) paused = mono.clone();
        else paused = cv::Mat();
        break;
    case 's':
        is.save();
        break;
    case 'q':
        return false;
    }
    return true;
 }



#ifdef USE_ROS

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

ros::Subscriber imageSub;

cv::Point trans(cv::Mat t, int x, int y)
{
    cv::Mat v(2, 1, CV_64F);
    v.at<double>(0, 0) = x;
    v.at<double>(1, 0) = y;
    v = t * v;
    return cv::Point(v.at<double>(0, 0), v.at<double>(1, 0));
}


void onImage(const sensor_msgs::ImageConstPtr & msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch(cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::Mat tim = cv_ptr->image, image = tim(cv::Rect(0, 0, tim.rows, 320));
    onImage(cv_ptr->image);


    /*std::unique_ptr<JacobiFrame> f(new JacobiFrame(image, 7));
    static std::unique_ptr<JacobiFrame> prev;
    static cv::Mat traj(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    static cv::Point oldPt(0, 0);
    static cv::Mat tr = cv::Mat::eye(2, 2, CV_64F), ps = cv::Mat::zeros(2, 1, CV_64F);
    if(prev == nullptr)
    {
        prev = std::move(f);
        return;
    }
    static bool updPrev = true;
    static bool pause = false;
    GN gnn;

    int lvl =  5;
    cv::Mat all, out;
    gnn.reset();
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
    cv::Point pt, cpt(320, 240);

    ps += tr * gnn.t_(cv::Rect(2, 0, 1, 2));
    tr *= gnn.t_(cv::Rect(0, 0, 2, 2));

    pt.x = (int)ps.at<double>(0, 0) * 0.2;
    pt.y = (int)ps.at<double>(1, 0) * 0.2;
    cv::line(traj, oldPt + cpt, pt + cpt, cv::Scalar(255, 0, 0));
    cv::Mat view = traj.clone();

    cv::circle(view, trans(tr, 5, 5) + pt + cpt, 4, cv::Scalar(0, 0, 255));
    cv::circle(view, trans(tr, -5, 5) + pt + cpt, 4, cv::Scalar(0, 0, 255));
    cv::circle(view, trans(tr, 5, -5) + pt + cpt, 4, cv::Scalar(0, 0, 0));
    cv::circle(view, trans(tr, -5, -5) + pt + cpt, 4, cv::Scalar(0, 0, 0));
    //cv::Mat copter(50. 50, CV_8UC3, cv::Scalar)

    cv::imshow("pos", view);
    oldPt = pt;
    if(all.empty()) all = out;
    cv::resize(all, all, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
    cv::imshow("all", all);

    char k = cv::waitKey(1);
    if(k == 'i')
        updPrev = !updPrev;
    if(k == 'p')
    {
        pause = !pause;
    }
    if(k == 'q') ros::shutdown();
    if(updPrev)
    {
        prev = std::move(f);
    }*/

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flyflow");
    ros::NodeHandle nh("~");

    imageSub = nh.subscribe("/camera/image_raw", 5, &onImage);

    ros::spin();
}

#else

void on_trackbar(int val, void * param)
{
 /*alpha = (double) alpha_slider/alpha_slider_max ;
 beta = ( 1.0 - alpha );

 addWeighted( src1, alpha, src2, beta, 0.0, dst);

 imshow( "Linear Blend", dst );*/
}

int main(int argc, char *argv[])
{
    //gradTest();
    cv::VideoCapture cap(1); // open the default camera
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(CV_CAP_PROP_FPS, 125);
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cv::Mat m = cv::imread("median.png", CV_LOAD_IMAGE_GRAYSCALE), invmed;
    double min, max;
    cv::minMaxLoc(m, &min, &max);
    m.convertTo(invmed, CV_8U, -1, max);
    conv.invmed_ = invmed;

    cv::namedWindow("tbs");
    cv::createTrackbar("contrast", "tbs", &contrast, 100, &on_trackbar);
    cv::createTrackbar("brightness", "tbs", &brightness, 100, &on_trackbar);

    for(;;)
    {
        cv::Mat image;
        cap >> image;

        /*image.rows = 240;
        image.cols = 320;
        cv::imshow("cam", image);*/

        if(!onImage(image)) return 0;
    }
    return 0;
}

#endif
