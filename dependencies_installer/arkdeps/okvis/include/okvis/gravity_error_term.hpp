#ifndef GRAVITY_ERROR_TERM_H_
#define GRAVITY_ERROR_TERM_H_

#include "Eigen/Core"
#include "ceres/autodiff_cost_function.h"

namespace okvis {


class PoseGraphGravityTerm {
 public:
  PoseGraphGravityTerm(const Eigen::Vector3d& g_measured,
                       const Eigen::Matrix<double, 3, 3>& sqrt_information)
      : g_measured_(g_measured), sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const q_a_ptr,
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);

    // get the estimated gravity vector, given current estimate of rotation
    Eigen::Matrix<T, 3, 1> g_estimated = q_a.inverse() * Eigen::Vector3d(0,0,1).template cast<T>();

    // Compute the residuals to the original gravity vector
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        g_estimated - g_measured_.template cast<T>();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ::ceres::CostFunction* Create(
      const Eigen::Vector3d& g_measured,
      const Eigen::Matrix<double, 3, 3>& sqrt_information) {
    return new ::ceres::AutoDiffCostFunction<PoseGraphGravityTerm, 3, 4>(
        new PoseGraphGravityTerm(g_measured, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The "measurement" of the gravity vector
  const Eigen::Vector3d g_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 3, 3> sqrt_information_;
};





}  // okvis

#endif  // GRAVITY_ERROR_TERM_H_