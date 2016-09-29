#ifndef IMUKALMAN_H
#define IMUKALMAN_H

#include "imu.h"





template<typename Time, typename Scalar>
class ImuKalmanPoint : public ImuPoint<Time, Scalar>
{
public:
    ImuKalmanPoint();
    Eigen::Matrix<Scalar, 5, 5> sigma; ///< Текущая ковариация процесса
    //template<class Owner>
    template<class Owner>
    void from(const std::pair<ImuKalmanPoint *, Owner *> &prev, const Owner *owner)
    {
        ImuPoint<Time, Scalar>::from(prev, owner);
/*


        sigma = G_ * prev.first->sigma * G_.transpose() + owner->matQ();


          double dt = state_new->time - state_old->time;

          // Noises.
          const Vector3 nav = Vector3::Constant(usercalc_.GetParamNoiseAcc());
          const Vector3 nbav = Vector3::Constant(usercalc_.GetParamNoiseAccbias());

          const Vector3 nwv = Vector3::Constant(usercalc_.GetParamNoiseGyr());
          const Vector3 nbwv = Vector3::Constant(usercalc_.GetParamNoiseGyrbias());

          // Bias corrected IMU readings.
          const Vector3 ew = state_new->w_m
              - state_new->template Get<StateDefinition_T::b_w>();
          const Vector3 ea = state_new->a_m
              - state_new->template Get<StateDefinition_T::b_a>();

          const Matrix3 a_sk = Skew(ea);
          const Matrix3 w_sk = Skew(ew);
          const Matrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

          const Matrix3 C_eq = state_new->template Get<StateDefinition_T::q>().toRotationMatrix();

          const double dt_p2_2 = dt * dt * 0.5;
          const double dt_p3_6 = dt_p2_2 * dt / 3.0;
          const double dt_p4_24 = dt_p3_6 * dt * 0.25;
          const double dt_p5_120 = dt_p4_24 * dt * 0.2;

          const Matrix3 Ca3 = C_eq * a_sk;
          const Matrix3 A = Ca3
              * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
          const Matrix3 B = Ca3
              * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
          const Matrix3 D = -A;
          const Matrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
          const Matrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
          const Matrix3 C = Ca3 * F;

          // Discrete error state propagation Matrix Fd according to:
          // Stephan Weiss and Roland Siegwart.
          // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
          // IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
          typename EKFState_T::F_type& Fd = state_old->Fd;

          enum {
            idxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                StateDefinition_T::p>::value,
            idxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                StateDefinition_T::v>::value,
            idxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                StateDefinition_T::q>::value,
            idxstartcorr_b_w = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                StateDefinition_T::b_w>::value,
            idxstartcorr_b_a = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                StateDefinition_T::b_a>::value
          };

          Fd.template block<3, 3>(idxstartcorr_p, idxstartcorr_v) = dt * eye3;
          Fd.template block<3, 3>(idxstartcorr_p, idxstartcorr_q) = A;

          Fd.template block<3, 3>(idxstartcorr_v, idxstartcorr_q) = C;

          Fd.template block<3, 3>(idxstartcorr_q, idxstartcorr_q) = E;

          typename EKFState_T::Q_type& Qd = state_old->Qd;

          CalcQCore<StateSequence_T, StateDefinition_T>(
              dt, state_new->template Get<StateDefinition_T::q>(), ew, ea, nav, nbav,
              nwv, nbwv, Qd);

          // Call user Q calc to fill in the blocks of auxiliary states.
          // TODO optim: make state Q-blocks map respective parts of Q using Eigen Map,
          // avoids copy.
          usercalc_.CalculateQAuxiliaryStates(*state_new, dt);

          // Now copy the userdefined blocks to Qd.
          boost::fusion::for_each(
              state_new->statevars,
              msf_tmp::CopyQBlocksFromAuxiliaryStatesToQ<StateSequence_T>(Qd));

          // TODO (slynen) Optim: Multiplication of F blockwise, using the fact that aux
          // states have no entries outside their block.
          state_new->P = Fd * state_old->P * Fd.transpose() + Qd; // !!!

          // Set time for best cov prop to now.
          time_P_propagated = state_new->time;

        }*/

        /*void MSF_Core<EKFState_T>::PropPToState(shared_ptr<EKFState_T>& state) {
          // Propagate cov matrix until the current states time.
          typename StateBuffer_T::iterator_T it = stateBuffer_.GetIteratorAtValue(
              time_P_propagated, false);
          typename StateBuffer_T::iterator_T itMinus = it;
          ++it;
          // Until we reached the current state or the end of the state list.
          for (; it != stateBuffer_.GetIteratorEnd() && it->second->time <= state->time;
              ++it, ++itMinus) {
            PredictProcessCovariance(itMinus->second, it->second);
          }
        } */

        /*template<class Derived>
        inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(
            const Eigen::MatrixBase<Derived> & vec) {
          EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
          return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
              vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
        }*/

    }
};

template<typename Scalar>
class ImuKalmanSensor : public ImuSensor<Scalar>
{
public:
    //const Eigen


};

#endif // IMUKALMAN_H
