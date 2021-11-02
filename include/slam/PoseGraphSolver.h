#pragma once

#include "util/Types.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/eigen.hpp>
#include <DBoW2.h>
#include <DLoopDetector.h>
#include <Eigen/Geometry>
#include "ceres/ceres.h"
#include <atomic>
#include <mutex>

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
            T* residuals_ptr) const;

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
