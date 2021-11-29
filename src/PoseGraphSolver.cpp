# include "slam/PoseGraphSolver.h"

namespace ark {


    GraphPose::GraphPose() :
        P_WA(0, 0, 0), Q_WA(Eigen::Matrix3d::Identity())
    {

    }

    GraphPose::GraphPose(const Eigen::Vector3d& P_WA, const Eigen::Quaterniond& Q_WA) :
        P_WA(P_WA), Q_WA(Q_WA)
    {

    }

    GraphPose::GraphPose(const Eigen::Matrix4d& T_WA)
    {
        P_WA = T_WA.block<3, 1>(0, 3);
        Q_WA = Eigen::Quaterniond(T_WA.block<3, 3>(0, 0));
    }

    PoseConstraint::PoseConstraint(int id_A, int id_B, const Eigen::Matrix4d& T_AB, const Eigen::Matrix<double, 6, 6>& sqrt_information /*= Eigen::Matrix<double, 6, 6>::Identity() */) :
        id_A(id_A), id_B(id_B), sqrt_information(sqrt_information)
    {
        P_AB = T_AB.block<3, 1>(0, 3);
        Q_AB = Eigen::Quaterniond(T_AB.block<3, 3>(0, 0));
    }

    PoseConstraint::PoseConstraint(int id_A, int id_B, const Eigen::Vector3d& P_AB, const Eigen::Quaterniond& Q_AB, const Eigen::Matrix<double, 6, 6>& sqrt_information /*= Eigen::Matrix<double, 6, 6>::Identity()*/) :
        id_A(id_A), id_B(id_B), P_AB(P_AB), Q_AB(Q_AB)
    {

    }

    PoseError::PoseError(const Eigen::Vector3d& p_ab_measured, const Eigen::Quaterniond& q_ab_measured, const Eigen::Matrix<double, 6, 6>& sqrt_information) :
        p_ab_measured_(p_ab_measured), q_ab_measured_(q_ab_measured),
        sqrt_information_(sqrt_information)
    {

    }

    PoseError::PoseError(const PoseConstraint& C_AB) :
        p_ab_measured_(C_AB.P_AB), q_ab_measured_(C_AB.Q_AB),
        sqrt_information_(C_AB.sqrt_information)
    {

    }

    ::ceres::CostFunction* PoseError::Create(const Eigen::Vector3d& p_ab_measured, const Eigen::Quaterniond& q_ab_measured, const Eigen::Matrix<double, 6, 6>& sqrt_information)
    {
        return new ::ceres::AutoDiffCostFunction<PoseError, 6, 3, 4, 3, 4>(
            new PoseError(p_ab_measured, q_ab_measured, sqrt_information));
    }

    ::ceres::CostFunction* PoseError::Create(const PoseConstraint& C_AB)
    {
        return new ::ceres::AutoDiffCostFunction<PoseError, 6, 3, 4, 3, 4>(
            new PoseError(C_AB));
    }


    SimplePoseGraphSolver::SimplePoseGraphSolver() :
        optimizing(false), loopQueued(false)
    {
        //set map pointer
    }

    void SimplePoseGraphSolver::AddConstraint(const PoseConstraint& constraint)
    {
        constraintMutex.lock();
        constraints_.push_back(constraint);
        constraintMutex.unlock();
    }

    void SimplePoseGraphSolver::AddConstraint(int id_A, int id_B, const Eigen::Vector3d& P_AB, const Eigen::Quaterniond& Q_AB, const Eigen::Matrix<double, 6, 6>& sqrt_information /*= Eigen::Matrix<double, 6, 6>::Identity()*/)
    {
        constraintMutex.lock();
        constraints_.push_back(PoseConstraint(id_A, id_B, P_AB, Q_AB, sqrt_information));
        constraintMutex.unlock();
    }

    void SimplePoseGraphSolver::AddConstraint(int id_A, int id_B, const Eigen::Matrix4d& T_AB, const Eigen::Matrix<double, 6, 6>& sqrt_information /*= Eigen::Matrix<double, 6, 6>::Identity()*/)
    {
        constraintMutex.lock();
        constraints_.push_back(PoseConstraint(id_A, id_B, T_AB, sqrt_information));
        constraintMutex.unlock();
    }

    void SimplePoseGraphSolver::AddPose(int id_A, const Eigen::Matrix4d& T_WA)
    {

        constraintMutex.lock();
        poses_.insert(std::pair<int, GraphPose>(id_A, T_WA));
        constraintMutex.unlock();
    }

    void SimplePoseGraphSolver::optimize()
    {
        if (optimizing) {
            loopQueued = true;
            return;
        }
        //optimizing=true;
        loopQueued = false;
        ::ceres::Problem problem;
        ::ceres::LossFunction* lossFunction = NULL;
        ::ceres::LocalParameterization* quaternion_local_parameterization =
            new ::ceres::EigenQuaternionParameterization;
        //add constraints from all keyframes
        constraintMutex.lock();
        for (size_t i = 0; i < constraints_.size(); i++) {
            ::ceres::CostFunction* costFunction =
                PoseError::Create(constraints_[i]);

            //Get Poses
            std::map<int, GraphPose>::iterator pose_A = poses_.find(constraints_[i].id_A);
            std::map<int, GraphPose>::iterator pose_B = poses_.find(constraints_[i].id_B);

            //Add residuals
            problem.AddResidualBlock(costFunction, lossFunction,
                pose_A->second.P_WA.data(), pose_A->second.Q_WA.coeffs().data(),
                pose_B->second.P_WA.data(), pose_B->second.Q_WA.coeffs().data());

            //Ensure proper quaternion parameterization
            problem.SetParameterization(pose_A->second.Q_WA.coeffs().data(),
                quaternion_local_parameterization);
            problem.SetParameterization(pose_B->second.Q_WA.coeffs().data(),
                quaternion_local_parameterization);
        }

        //Set start pose as known
        std::map<int, GraphPose>::iterator poseStart = poses_.begin();
        problem.SetParameterBlockConstant(poseStart->second.P_WA.data());
        problem.SetParameterBlockConstant(poseStart->second.Q_WA.coeffs().data());

        ::ceres::Solver::Options options;
        options.max_num_iterations = 10;
        //options.linear_solver_type = ::ceres::SPARSE_NORMAL_CHOLESKY;
        options.linear_solver_type = ::ceres::SPARSE_SCHUR;
        options.trust_region_strategy_type = ::ceres::DOGLEG;
        options.num_threads = 2;

        ::ceres::Solver::Summary summary;
        ::ceres::Solve(options, &problem, &summary);
        constraintMutex.unlock();

        //std::cout << summary.FullReport() << '\n';
    }

    Eigen::Matrix4d SimplePoseGraphSolver::getTransformById(int id)
    {
        //Eigen::Affine3d outT = poses_[id].Q_WA*Eigen::Translation3d(poses_[id].P_WA);
        //return outT.matrix();
        Eigen::Matrix4d outMat = Eigen::Matrix4d::Identity();
        outMat.block<3, 1>(0, 3) = poses_[id].P_WA;
        outMat.block<3, 3>(0, 0) = poses_[id].Q_WA.toRotationMatrix();
        return outMat;
    }

}