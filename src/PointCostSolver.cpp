# include "PointCostSolver.h"


namespace ark {

    template <typename PointT>
    ark::PointCostFunctor<PointT>::PointCostFunctor(const CloudPtr src, const CloudPtr tgt, const std::vector<int>& correspondences, const std::vector<bool>& inliers) :src_(src),
        tgt_(tgt),
        correspondences_(correspondences),
        inliers_(inliers)
    {

    }

    template <typename T>
    bool ark::PointCostFunctor::operator()(const T* const rotation_ptr, const T* const translation_ptr, T* residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > translation(translation_ptr);
        Eigen::Quaternion<T> rotation(rotation_ptr[3], rotation_ptr[0], rotation_ptr[1], rotation_ptr[2]);
        for (size_t i = 0; i < src_->size(); i++) {
            if (inliers_[i]) {
                Eigen::Matrix<T, 3, 1> s_x(
                    static_cast<T>(src_->points[i].x),
                    static_cast<T>(src_->points[i].y),
                    static_cast<T>(src_->points[i].z));
                Eigen::Matrix<T, 3, 1> t_x(
                    static_cast<T>(tgt_->points[correspondences_[i]].x),
                    static_cast<T>(tgt_->points[correspondences_[i]].y),
                    static_cast<T>(tgt_->points[correspondences_[i]].z));
                Eigen::Matrix<T, 3, 1> r_vec = t_x - (rotation*s_x + translation);
                residuals_ptr[3 * i] = r_vec[0];
                residuals_ptr[(3 * i) + 1] = r_vec[1];
                residuals_ptr[(3 * i) + 2] = r_vec[2];
            }
            else {
                residuals_ptr[3 * i] = (T)0;
                residuals_ptr[3 * i + 1] = (T)0;
                residuals_ptr[3 * i + 2] = (T)0;
            }
            //}
        }
        return true;
    }

    template <typename PointT>
    Eigen::Affine3d ark::PointCostSolver<PointT>::solve(const CloudPtr src, const CloudPtr tgt, const std::vector<int>& correspondences, const std::vector<bool>& inliers, const Eigen::Affine3d& initial_pose_estimate, int max_num_iterations /*= 20*/)
    {
        ceres::Problem problem;
        Eigen::Quaterniond ceres_rotation(initial_pose_estimate.rotation());
        Eigen::Vector3d ceres_translation(initial_pose_estimate.translation());
        problem.AddParameterBlock(ceres_rotation.coeffs().data(), 4,
            std::unique_ptr<ceres::LocalParameterization>(
                new ceres::EigenQuaternionParameterization).release());
        problem.AddParameterBlock(ceres_translation.data(), 3, nullptr);

        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PointCostFunctor<PointT>,
            ceres::DYNAMIC, 4, 3>(
                new PointCostFunctor<PointT>(src, tgt, correspondences, inliers),
                3 * src->size()),
            nullptr, ceres_rotation.coeffs().data(), ceres_translation.data());

        ceres::Solver::Options options;
        options.max_num_iterations = max_num_iterations;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Affine3d outTrans = Eigen::Translation3d(ceres_translation)*ceres_rotation;
        return outTrans;
    }


    template <typename PointT>
    ark::PointCostSolver<PointT>::PointCostSolver()
    {

    }
}