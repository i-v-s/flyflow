#ifndef ELLIPSOIDCALIBRATOR_H
#define ELLIPSOIDCALIBRATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>


template<class Scalar>
class EllipsoidCalibrator
{
private:
    Eigen::Matrix<Scalar, 6, 6> h2; // H^T * H
    Eigen::Matrix<Scalar, 6, 1> htw;   // H^T * w
public:   
    EllipsoidCalibrator() { reset(); }
    void reset()
    {
        h2.setZero();
        htw.setZero();
    }
    template<class ValueType>
    void addValue(const ValueType *mag)
    {
        Eigen::Matrix<Scalar, 6, 1> hnt; // n-th row of matrix H, transposed
        int x = mag[0], y = mag[1], z = mag[2];
        hnt << x, y, z, -y * y, -z * z, 1;
        h2 += hnt * hnt.transpose();
        htw += hnt * (x * x);
    }
    template<class ResultType>
    void solve(ResultType *offset, ResultType *scale)
    {
        Eigen::Matrix<Scalar, 6, 1> x = h2.colPivHouseholderQr().solve(htw);
        offset[0] = x[0] * 0.5f;
        offset[1] = x[1] / (2.0f * x[3]);
        offset[2] = x[2] / (2.0f * x[4]);
        Scalar a = x[5] + offset[0] * offset[0] + x[3] * offset[1] * offset[1] + x[4] * offset[2] * offset[2];
        Scalar b = a / x[3];
        Scalar c = a / x[4];
        scale[0] = sqrt(a);
        scale[1] = sqrt(b);
        scale[2] = sqrt(c);
    }
};







#endif // ELLIPSOIDCALIBRATOR_H
