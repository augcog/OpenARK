#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/eigen.hpp>
#include <DBoW2.h>
#include <DLoopDetector.h>
#include <Eigen/Geometry>
#include <atomic>
#include <mutex>
#include "util/Types.h"
#include "ceres/ceres.h"

namespace ark{

class GraphPose {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GraphPose();
    GraphPose(const Eigen::Vector3d& P_WA, 
            const Eigen::Quaterniond& Q_WA);

    GraphPose(const Eigen::Matrix4d& T_WA);

    Eigen::Vector3d P_WA;
    Eigen::Quaterniond Q_WA; 
};

class PoseConstraint{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseConstraint(int id_A, int id_B, const Eigen::Matrix4d& T_AB, 
            const Eigen::Matrix<double, 6, 6>& sqrt_information =
                Eigen::Matrix<double, 6, 6>::Identity() );

    PoseConstraint(int id_A, int id_B, const Eigen::Vector3d& P_AB, 
            const Eigen::Quaterniond& Q_AB, 
            const Eigen::Matrix<double, 6, 6>& sqrt_information =
                Eigen::Matrix<double, 6, 6>::Identity());

    int id_A, id_B;
    Eigen::Vector3d P_AB;
    Eigen::Quaterniond Q_AB; 
    Eigen::Matrix<double, 6, 6> sqrt_information;
}; //PoseConstraint

class PoseError {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseError(const Eigen::Vector3d& p_ab_measured, 
            const Eigen::Quaterniond& q_ab_measured, 
            const Eigen::Matrix<double, 6, 6>& sqrt_information);

    PoseError(const PoseConstraint& C_AB);

    //Evaluation of the error term
    template <typename T>
    bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
        const T* const p_b_ptr, const T* const q_b_ptr,
        T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T> > q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
            q_ab_measured_.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
            p_ab_estimated - p_ab_measured_.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ::ceres::CostFunction* Create(
            const Eigen::Vector3d& p_ab_measured, 
            const Eigen::Quaterniond& q_ab_measured, 
            const Eigen::Matrix<double, 6, 6>& sqrt_information);

    static ::ceres::CostFunction* Create(
            const PoseConstraint& C_AB);


private:
    // The measurement for the position of B relative to A in the A frame.
    const Eigen::Vector3d p_ab_measured_;
    // The measurement for the rotation of B relative to A in the A frame.
    const Eigen::Quaterniond q_ab_measured_; 
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
}; //PoseConstraint

class SimplePoseGraphSolver{
public:
    SimplePoseGraphSolver();

    void AddConstraint(const PoseConstraint& constraint);;

    void AddConstraint(int id_A, int id_B, const Eigen::Vector3d& P_AB, const Eigen::Quaterniond& Q_AB, 
            const Eigen::Matrix<double, 6, 6>& sqrt_information = Eigen::Matrix<double, 6, 6>::Identity());;

    void AddConstraint(int id_A, int id_B, const Eigen::Matrix4d& T_AB,
            const Eigen::Matrix<double, 6, 6>& sqrt_information = Eigen::Matrix<double, 6, 6>::Identity());;

    void AddPose(int id_A, const Eigen::Matrix4d& T_WA);

    void optimize();

    Eigen::Matrix4d getTransformById(int id);

    std::vector<PoseConstraint> constraints_;
    std::atomic<bool> optimizing;
    std::atomic<bool> loopQueued;
    std::mutex constraintMutex;
    std::map<int, GraphPose> poses_;


    //re-build optimization problem
    //solve optimization problem
    //optimization loop
    //notify when done
    //map pointer
    //updatemap
    //recursively update remiander of map
};//PoseGraphSolver

}//ark
