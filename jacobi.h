#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace flyflow
{

class JacobiLevel
{
private:
    int w_, h_;
    const float ks_, ks2_;
    cv::Mat f; // Image
    cv::Mat gx_, gy_; // Gradients
    cv::Mat gxx_, gyy_, gxy_, gyx_;
    cv::Mat a_;
    cv::Mat * j_[6];
public:
    JacobiLevel(int w, int h, int level);
    void set(const JacobiLevel & src);
    void set(const cv::Mat & f1);
    bool solve(const cv::Mat & error, cv::Mat & transf);
};

class Jacobi
{
private:
    std::vector<JacobiLevel> levels_;
public:
    Jacobi(int w, int h, int min = 10);
    void set(const cv::Mat & image);
};

}
#endif // FRAME_H
