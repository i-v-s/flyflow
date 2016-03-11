#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Geometry>

#include "conveyor.h"
#include "visualizer.h"
#include "imagestat.h"
#include "kalman.h"

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

Eigen::Matrix3d caMat;// = Eigen::Matrix3d::Identity();

class Feature
{
public:
    Eigen::Matrix<double, 10, 10> sigma_;
    Eigen::Vector3d pos_;

    cv::Point2f pt;
    int age;
    cv::Scalar col;
    void diffQuatMul(Eigen::Vector3d * v,
                  Eigen::Matrix<double, 3, 3> * dVdA,
                  Eigen::Matrix<double, 3, 4> * dVdQ,
                  const Eigen::Quaterniond & q,
                  const Eigen::Vector3d & a)
    {
        Eigen::Quaterniond qc = q.conjugate();
        Eigen::Matrix3d rm = qc.toRotationMatrix();
        *v = rm * a;
        *dVdA = rm;

        double _2x = 2 * qc.x(), _2y = 2 * qc.y(), _2z = 2 * qc.z(), _2w = 2 * qc.w();
        Eigen::Matrix3d dRdx, dRdy, dRdz, dRdw;
        dRdx <<      0,    _2y,    _2z,
                   _2y, -2*_2x,   -_2w,
                   _2z,    _2w, -2*_2x;
        dVdQ->block<3, 1>(0, 0) = -dRdx * a;

        dRdy << -2*_2y,    _2x,    _2w,
                   _2x,      0,    _2z,
                  -_2w,    _2z, -2*_2y;
        dVdQ->block<3, 1>(0, 1) = -dRdy * a;

        dRdz << -2*_2z,   -_2w,    _2x,
                   _2w, -2*_2z,    _2y,
                   _2x,    _2y,      0;
        dVdQ->block<3, 1>(0, 2) = -dRdz * a;

        dRdw <<      0,   -_2z,    _2y,
                   _2z,      0,   -_2x,
                  -_2y,    _2x,      0;
        dVdQ->block<3, 1>(0, 3) = dRdw * a;
    }
    template<int n> void diffDiv(Eigen::Matrix<double, 2, n> * dXYdV, const Eigen::Vector3d & v)
    {

    }

    bool h(Eigen::Matrix<double, 2, 1> * Z, Eigen::Matrix<double, 2, 10> * H, const Eigen::Matrix<double, 10, 1> & X)
    {
        Eigen::Vector3d a = X.block<3, 1>(7, 0) - X.block<3, 1>(4, 0);
        Eigen::Quaterniond q = Eigen::Map<const Eigen::Quaterniond>(X.data()).normalized();
        //q.normalize();

        Eigen::Matrix3d rm;
        Eigen::Matrix<double, 3, 4> dVdQ;
        Eigen::Vector3d v;
        diffQuatMul(&v, &rm, &dVdQ, q, a);
        v = caMat * v;
        dVdQ = caMat * dVdQ;

        Eigen::Matrix3d mr = caMat * rm;
        if(v(2) < 0.01) return false;
        (*Z)(0) = v(0) / v(2);
        (*Z)(1) = v(1) / v(2);
        // Derivative of screen coords on copter pos
        Eigen::Matrix<double, 2, 3> dxydP = (mr.block<2, 3>(0, 0) * v(2) - v.block<2, 1>(0, 0) * mr.block<1, 3>(2, 0)) / (v(2) * v(2));
        H->block<2, 3>(0, 4) = -dxydP;
        H->block<2, 3>(0, 7) =  dxydP;
        Eigen::Matrix<double, 2, 4> dxydQ = (dVdQ.block<2, 4>(0, 0) * v(2) - v.block<2, 1>(0, 0) * dVdQ.block<1, 4>(2, 0)) / (v(2) * v(2));
        dxydQ *= (Eigen::Matrix<double, 4, 4>::Identity() - q.coeffs() * q.coeffs().transpose());
        H->block<2, 4>(0, 0) = dxydQ;

        return true;
    }
    void testdqm()
    {
        Eigen::Vector3d a;
        a << 2, 3, 5;
        Eigen::Quaterniond q(10, -7, 1, 4);
        q.normalize();

        Eigen::Vector3d V;
        Eigen::Vector2d S;
        Eigen::Matrix<double, 3, 3> dVdA;
        Eigen::Matrix<double, 3, 4> dVdQ;
        Eigen::Matrix<double, 2, 4> dSdQ;

        diffQuatMul(&V, &dVdA, &dVdQ, q, a);
        {
            V = caMat * V;
            dVdQ = caMat * dVdQ;
            S(0) = V(0) / V(2);
            S(1) = V(1) / V(2);
            dSdQ = (dVdQ.block<2, 4>(0, 0) * V(2) - V.block<2, 1>(0, 0) * dVdQ.block<1, 4>(2, 0)) / (V(2) * V(2));
            for(int x = 0; x < 4; x++)
            {
                Eigen::Quaterniond q1 = q;
                q1.coeffs()(x) += 0.001;
                //Eigen::Quaterniond q2 = q1.normalized();
                q1.normalize();
                Eigen::Vector3d V1 = caMat * (q1.conjugate()._transformVector(a));

                Eigen::Matrix<double, 3, 1> dVr = V1 - V;
                Eigen::Matrix<double, 3, 1> dVj = dVdQ * (q1.coeffs() - q.coeffs());
                std::cout << "dVr " << x << " = " << dVr.transpose() << std::endl;
                std::cout << "dVj " << x << " = " << dVj.transpose() << std::endl;
                //std::cout << "e" << x << " = " << (dZ2 - dZ1).transpose() << std::endl;

                Eigen::Vector2d S1;
                S1(0) = V1(0) / V1(2);
                S1(1) = V1(1) / V1(2);

                Eigen::Matrix<double, 2, 1> dSr = S1 - S;
                Eigen::Matrix<double, 2, 1> dSj = dSdQ * (q1.coeffs() - q.coeffs());
                std::cout << "dSr " << x << " = " << dSr.transpose() << std::endl;
                std::cout << "dSj " << x << " = " << dSj.transpose() << std::endl;


            }

        }
    }
    void testh()
    {

        Eigen::Matrix<double, 2, 1> Z, Z1, Z2;
        Eigen::Matrix<double, 2, 10> H, H1, Hr;
        Eigen::Matrix<double, 10, 1> X;
        X << 0, 0, 1, 0, // Copter quaternion
             1, 0, 1, // Copter pos
             3, 1, 4; // Feature pos
        if(h(&Z, &H, X))
        {
            std::cout << "Z = " << Z.transpose() << std::endl;
            std::cout << "H = " << std::endl << H << std::endl;
            for(int x = 0; x < 10; x++)
            {
                Eigen::Matrix<double, 10, 1> X1 = X;
                X1(x) += 0.001;
                h(&Z1, &H1, X1);
                Hr.block<2, 1>(0, x) = (Z1 - Z) / 0.001;
            }
            std::cout << "Hr = " << std::endl << Hr << std::endl;
        }
    }
};


class Tracker
{
public:
    Kalman<double, 7> kalman_;
    Corrector<double, Feature, 10, 2> corrector_;

    cv::Ptr<cv::Feature2D> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    std::vector<cv::KeyPoint> lastkp_;
    cv::Mat lastdesc_;
    cv::Mat lastimg_;
    typedef std::map<int, std::unique_ptr<Feature> > Map;
    std::unique_ptr<Map> features_;

    Tracker():
        detector_(cv::ORB::create()),
        //matcher_(cv::DescriptorMatcher::create("FlannBased"))
        matcher_(cv::DescriptorMatcher::create("BruteForce-Hamming"))
    {


    }

    void onImage(const cv::Mat & image)
    {
        image -= 80;
        image *= 3;
        cv::Mat desc;
        std::vector<cv::KeyPoint> kp;
        detector_->detectAndCompute(image, cv::noArray(), kp, desc);
        if(kp.empty()) return;
        if(!lastkp_.empty())
        {
            kalman_.predict();
            std::vector<std::vector<cv::DMatch>> matches;
            //std::vector<cv::KeyPoint> m1, m2;
            matcher_->knnMatch(lastdesc_, desc, matches, 2);
            if(matches.empty()) return;
            cv::Mat res;
            std::vector<cv::DMatch> good_matches;

            std::unique_ptr<Map> nf(new Map());
            for(int i = 0; i < matches.size(); i++)
            {
                float ratio = 0.8;
                if(matches[i][0].distance < ratio * matches[i][1].distance)
                {
                    cv::DMatch & m = matches[i][0];
                    good_matches.push_back(matches[i][0]);
                    if(features_ == nullptr) continue;
                    Map::iterator mi = features_->find(m.queryIdx);
                    std::unique_ptr<Feature> f;
                    if(mi == features_->end())
                    {
                        f = std::unique_ptr<Feature>(new Feature());
                        f->age = 0;
                        f->col = cv::Scalar(std::rand() & 255, std::rand() & 255, std::rand() & 255);
                    }
                    else
                        f = std::move(mi->second);
                    Eigen::Vector2d z;
                    cv::Point2f pt = kp[m.trainIdx].pt;
                    z(0) = pt.x;
                    z(1) = pt.y;
                    f->pt = pt;
                    f->age++;
                    Eigen::Matrix<double, 10, 1> X;
                    X << kalman_.X_, f->pos_;
                    corrector_.correct(X, f->sigma_, z, f.get());
                    nf->insert(std::make_pair(m.trainIdx, std::move(f)));
                }
            }
            cv::cvtColor(image, res, CV_GRAY2RGB);
            for(Map::iterator i = nf->begin(); i != nf->end(); i++)
            {
                cv::Point2f p = i->second->pt;
                cv::circle(res, p, i->second->age, i->second->col);
            }

            //cv::drawMatches(lastimg_, lastkp_, image, kp, good_matches, res, cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0));
            cv::imshow("res", res);
            features_ = std::move(nf);
        }
        lastkp_ = kp;
        lastdesc_ = desc;
        lastimg_ = image;
    }
};

Tracker tr;

bool onImage(const cv::Mat & image)
{
    /*cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> kp;
    orb->detect(image, kp);
    cv::Mat desc;
    orb->compute(image, kp, desc);

     //= surf.detectAndCompute(image, cv::None);
    cv::Mat im2;
    cv::drawKeypoints(image, kp, im2);
    cv::Ptr<cv::FlannBasedMatcher> m = cv::FlannBasedMatcher::create();
    m->knnMatch();


    cv::imshow("orb", im2);*/
    tr.onImage(image);

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
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

ros::Subscriber imageSub, imuSub;

cv::Point trans(cv::Mat t, int x, int y)
{
    cv::Mat v(2, 1, CV_64F);
    v.at<double>(0, 0) = x;
    v.at<double>(1, 0) = y;
    v = t * v;
    return cv::Point(v.at<double>(0, 0), v.at<double>(1, 0));
}

struct Imu
{
    double time;
    double a[3];
    double w[3];
};

void onImu(const sensor_msgs::ImuConstPtr & msg)
{

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

class Test
{
public:
    int n;
    Test(int n): n(n) { std::cout << "Test " << n << " created" << std::endl;}
    void show() { std::cout << "Test " << n << " show" << std::endl;}
    ~Test() { std::cout << "Test " << n << " destroyed" << std::endl;}
};


int main(int argc, char** argv)
{
    caMat << 300,   0, 320,
               0, 300, 240,
               0,   0,   1;
    Feature f;
    f.testdqm();
    f.testh();

    ros::init(argc, argv, "flyflow");
    ros::NodeHandle nh("~");

    imageSub = nh.subscribe("/camera/image_raw", 5, &onImage);

    imuSub = nh.subscribe("/imu/data", 5, &onImu);

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
