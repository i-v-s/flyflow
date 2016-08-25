#pragma once
#include "sequenceloop.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

template<typename Time, typename Scalar>
class ImuPoint
{
    Time time_;
public:
    inline Time time() const { return time_;}
    typedef Eigen::Quaternion<Scalar> Quaternion;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
    struct Measure
    {
        Vector3 a, w; // Линейное ускорение, угловая скорость
    } measure;
    struct State
    {
        Quaternion q;
        Vector3 v;
        Vector3 p;
    } state;
    ImuPoint(Time t, const Measure &m) : time_(t), measure(m) {}
    template<class Derived>
    static inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatJPL( const Eigen::MatrixBase<Derived> & vec ) {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
      return (Eigen::Matrix<typename Derived::Scalar, 4, 4>() << 0, vec[2], -vec[1],
          vec[0], -vec[2], 0, vec[0], vec[1], vec[1], -vec[0], 0, vec[2], -vec[0],
          -vec[1], -vec[2], 0)
          .finished();
    }
    template<class Pair, class Owner>
    void from(const Pair &prev, const Owner *owner)
    {
        Time dt = time() - prev.first->time();
        Vector3 wPrev = prev.first->measure.w - owner->wBias();
        Vector3 wNew  = measure.w - owner->wBias();
        Vector3 wMean = (wPrev + wNew) * 0.5; // Усредняем и интегрируем

        Matrix4 omegaMean = omegaMatJPL(wMean), omegaNew = omegaMatJPL(wNew), omegaPrev = omegaMatJPL(wPrev);

        int div = 1;
        Matrix4 matExp;
        matExp.setIdentity();
        omegaMean *= 0.5 * dt;
        for (int i = 1; i < 5; i++)
        {
            div *= i;
            matExp = matExp + omegaMean / div;
            omegaMean *= omegaMean;
        }

        const Matrix4 quat_int =
            matExp + 1.0 / 48.0 * (omegaNew * omegaPrev - omegaPrev * omegaNew) * dt * dt;

        state.q.coeffs() = quat_int * prev.first->state.q.coeffs(); // вращение new.q = (E + Sum((W * dt / 2) ^ n / n ) ) * old.q - интегр. до n порядка
        state.q.normalize();

        Vector3 aNew = measure.a - owner->aBias();
        Vector3 aOld = prev.first->measure.a - owner->aBias();
        Vector3 dv = (state.q.toRotationMatrix() * aNew + prev.first->state.q.toRotationMatrix() * aOld) / 2; // ускорение dv = (new.q * ea + old.q * eaold) / 2

        state.v = prev.first->state.v + (dv - owner->gravity()) * dt;
        state.p = prev.first->state.p + (state.v + prev.first->state.v) * (dt / 2);
    }
    template<class Owner>
    void from(const Owner *)
    {
        state.q.setIdentity();
        state.p.setZero();
        state.v.setZero();
    }
};


/*void Copter::f(PoseRef &d, const PoseRef &X)
{
    const itg::real & wx = w[0], & wy = w[1], & wz = w[2];
    const itg::Ref & Q = X.Q;

    d.Q[0] =              wz * Q[1] - wy * Q[2] + wx * Q[3];
    d.Q[1] = -wz * Q[0]             + wx * Q[2] + wy * Q[3];
    d.Q[2] =  wy * Q[0] - wx * Q[1]             + wz * Q[3];
    d.Q[3] = -wx * Q[0] - wy * Q[1] - wz * Q[2]            ;
}
*/

template<typename Time = double, typename Scalar = double, int N = 64>
class ImuSequence : public SequenceLoop<ImuPoint<Time, Scalar>, N>
{
    typedef Eigen::Quaternion<Scalar> Quaternion;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
    Vector3 wBias_, aBias_, gravity_;
public:
    inline void setWBias(const Vector3 &wb) { wBias_ = wb; }
    inline void setGravity(Scalar gravity) { gravity_ << 0, 0, gravity; }
    inline const Vector3 &wBias() const { return wBias_; }
    inline const Vector3 &aBias() const { return aBias_; }
    inline const Vector3 &gravity() const { return gravity_; }
    //void predict(T time);
    ImuSequence()
    {
        aBias_.setZero();
        wBias_.setZero();
    }

};
