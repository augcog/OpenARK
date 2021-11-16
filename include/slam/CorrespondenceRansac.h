#pragma once

#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>

namespace ark{

template<typename PointT>
class CorrespondenceRansac{
    using CloudPtr = typename pcl::PointCloud<PointT>::Ptr;
public:

    //This function performs ransac on a set of feature
    //correspondences to determince which are inliers
    //also returns an estimate of the transform between clouds
    static void getInliersWithTransform(
            CloudPtr src, 
            CloudPtr tgt,
            const std::vector<int>& correspondences,
            int num_samples, 
            float inlier_threshold, 
            int num_iterations,
            int& num_inliers_out,
            std::vector<bool>& inliers_out,
            Eigen::Affine3d transform_out);   

private:

    //Checks transformed points against threshold
    static bool checkThreshold(const PointT& p1, const PointT& p2, float inlier_threshold);

    CorrespondenceRansac();
};



}//namespace ICP

