#ifndef UTILS_H
#define UTILS_H

namespace flyflow
{

typedef float scalar;
enum {cv_scalar = CV_32F};


template<class T> void shrink(const cv::Mat & src, cv::Mat & dst)
{
    assert(src.cols / 2 == dst.cols && src.rows / 2 == dst.rows);
    const T * s = (const T *) src.data;
    T * d = (T *) dst.data;
    int w = src.cols, h = src.rows;
    for(int y = (h >> 1); y--;)
    {
        for(int x = (w >> 1); x--;)
        {
            *(d++) = s[0] + s[1] + s[w] + s[w + 1];
            s += 2;
        }
        s += w;
    }
}

template<class Ts, class Td = Ts> void mulByX(const cv::Mat & src, cv::Mat & dst)
{
    assert(src.rows == dst.rows && src.cols == dst.cols);
    const Ts * s = (const Ts *) src.data;
    Td * d = (Td *) dst.data;
    for(int h = src.rows, w = src.cols; h--; )
        for(int x = 0; x < w; x++)
            *(d++) = (*(s++) * x);// / ((w >> 4) + 1);
}

template<class Ts, class Td = Ts> void mulByY(const cv::Mat & src, cv::Mat & dst)
{
    assert(src.rows == dst.rows && src.cols == dst.cols);
    const Ts * s = (const Ts *) src.data;
    Td * d = (Td *) dst.data;
    for(int y = 0, h = src.rows, w = src.cols; y < h; y++)
        for(int x = w; x--; )
            *(d++) = (*(s++) * y);// / ((h >> 4) + 1);
}

template<class T1, class T2 = T1> static inline double scalMul(const cv::Mat & m1, const cv::Mat & m2)
{
    assert(m1.rows == m2.rows && m2.cols == m2.cols);
    double sum = 0;
    const T1 * a = (const T1 *) m1.data;
    const T2 * b = (const T2 *) m2.data;
    for(const T1 * end = (const T1 *)m1.dataend; a < end; a++, b++)
        sum += double(*a) * double(*b);
    return sum;
}

}
#endif // UTILS_H
