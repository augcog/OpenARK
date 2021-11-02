#pragma once

#include <vector>
#include <Eigen/Core>
#include "ceres/ceres.h"
#include "pcl/point_cloud.h"

namespace ark{

template <typename PointT>
class PointCostFunctor {
private:
    using CloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    const CloudPtr src_; 
    const CloudPtr tgt_;
    const std::vector<int> correspondences_;
    const std::vector<bool> inliers_;

public:
    explicit PointCostFunctor(
        const CloudPtr src, 
        const CloudPtr tgt, 
        const std::vector<int>& correspondences,
        const std::vector<bool>& inliers);

    PointCostFunctor(const PointCostFunctor&) = delete;
    PointCostFunctor& operator=(const PointCostFunctor&) = delete;

    template <typename T>
    bool operator()(const T* const rotation_ptr, const T* const translation_ptr, T* residuals_ptr) const;
};

template <typename PointT>
class PointCostSolver{
    using CloudPtr = typename pcl::PointCloud<PointT>::Ptr;
public:
    static Eigen::Affine3d solve(
            const CloudPtr src,
            const CloudPtr tgt, 
            const std::vector<int>& correspondences, 
            const std::vector<bool>& inliers,
            const Eigen::Affine3d& initial_pose_estimate,
            int max_num_iterations = 20);
private:
    PointCostSolver();
};
}//namespace ICP
