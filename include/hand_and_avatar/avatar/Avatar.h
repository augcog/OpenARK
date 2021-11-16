#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <ceres/ceres.h>
#include "Version.h"
#include "util/nanoflann.hpp"
#include "util/Visualizer.h"

namespace ark {
    class HumanDetector;
    struct HumanAvatarUKFModel;

    /** Gaussian Mixture Model */
    class GaussianMixture {        
    public:
        /** load Gaussian Mixture parameters from 'path' */
        void load(const std::string & path);

        /** get number of Gaussian mixture components */
        int numComponents() const;

        template<class T>
        /** Compute PDF at 'input' */
        T pdf(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x) const;

        template<class T>
        /** Compute Ceres residual vector (squaredNorm of output vector is equal to min_i -log(c_i pdf_i(x))) */
        Eigen::Matrix<T, Eigen::Dynamic, 1> residual(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x);
    private:
        int nComps, nDims;
        Eigen::VectorXd weight;
        Eigen::MatrixXd mean;
        std::vector<Eigen::MatrixXd> cov;

        // leading constants
        Eigen::VectorXd consts, consts_log;
        // cholesky decomposition of inverse: cov^-1 = cov_cho * cov_cho^T
        std::vector<Eigen::MatrixXd> cov_cho;
    };

    /** Represents a humanoid avatar */
    class HumanAvatar {
    public:
        typedef pcl::PointXYZRGBA Point_T;
        typedef pcl::PointCloud<Point_T> Cloud_T;

        /** Eigen "point cloud" representation. Contains 3 columns, one point per row */
        typedef Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> EigenCloud_T;
        typedef Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> EigenCloud2d_T;

        /** Names for the various skeletal joints in the SMPL model (does not work for other models) */
        typedef enum _JointType {
            // TODO: delegate to a skeleton instead

            // BFS Order (topologically sorted)
            ROOT_PELVIS = 0, L_HIP, R_HIP, SPINE1, L_KNEE, R_KNEE, SPINE2, L_ANKLE,
            R_ANKLE, SPINE3, L_FOOT, R_FOOT, NECK, L_COLLAR, R_COLLAR, HEAD, L_SHOULDER,
            R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST, L_HAND, R_HAND,

            _COUNT
        } JointType;
        
        /* sizes of data types (in bytes) for each parameter vector type */
        static const int NUM_ROT_PARAMS = 4, NUM_ROT_MAT_PARAMS = 9, NUM_POS_PARAMS = 3;

        /* weight key, joint numbers. This allows us to use static cost function with Ceres. */
        static const int NUM_SHAPEKEYS = 10, NUM_JOINTS = HumanAvatar::JointType::_COUNT;

        /** maximum number of joints assigned to each bone (max # bone weights) */
        static const int MAX_ASSIGNED_JOINTS = 8;

        /** Represents a joint in the model's skeleton. Contains rotation information about bone ending at the joint.
         * Mostly a virtualized structure and data is largely stored in parameter arrays in the Avatar class itself. */
        struct Joint {
            typedef std::shared_ptr<Joint> Ptr;

            /** The associated avatar */
            HumanAvatar & avatar;

            /** The name of the joint */
            std::string name;

            /** The JointType (= index in joints vector) */
            JointType type;

            /** The parent joint; nullptr for root */
            Joint * parent = nullptr;

            /** A list of child joints */
            std::vector<Joint *> children;
            Eigen::Vector3d posSkel;                 // (defined) base position of joint from skeleton file
            Eigen::Map<Eigen::Vector3d> posBase,     // (computed) current base position of joint, accounting for shape keys
                                    posTransformed;  // (computed) current global position of joint
            Eigen::Map<Eigen::Quaterniond> rotTransformed, // (computed) current global rotation of bone ending at joint
                                           rotation;       // (PARAMETER) local rotation of bone ending at joint, set by user
                         
            Eigen::Map<Eigen::Matrix3d> cachedTransform; // cached overall transformation

            /** Contruct a joint associated with the given avatar & joint type */
            Joint(HumanAvatar & avatar, JointType type);
        };

        /** The directory the avatar's model was imported from */
        const std::string MODEL_DIR;

        /** Create an avatar from the model information in 'model_dir'
         *  @param model_dir path to directory containing model files
         */
        explicit HumanAvatar(const std::string & model_dir, int downsample_factor = 1);

        /** Create an avatar from the model information in 'model_dir'
         *  @param model_dir path to directory containing model files
         *  @param shape_keys names of shape keys in 'model_dir'/shapekey to use
         */
        HumanAvatar(const std::string & model_dir, const std::vector<std::string> & shape_keys, double downsample_radius = 0.001);

        /** Destructor for HumanAvatar */
        ~HumanAvatar();

        // section GETTERS/SETTERS

        // model parameter vector getters/setters
        /** Get the avatar's key weight parameter vector
          * (ith index is relative weight of ith shape key a.k.a. blendshape;
          *  total size is numKeys()) */
        double * w();
        const double * w() const;

        /** Get the avatar's bone rotation parameter vector (stores quaternions: x y z w)
          * (first 4 indices are root rotation quaternion, total size is numJoints() * 4) */
        double * r();
        const double * r() const;

        /** Get the avatar's global position (length 3) */
        double * p();
        const double * p() const;

        /** Compute the avatar's SMPL pose parameters (Rodrigues angles) */
        Eigen::VectorXd smplParams() const;

        // utility functions for modifying the pose

        /* Get global position of the skeleton's center */
        Eigen::Map<Eigen::Vector3d> getBasePosition();

        /** Get the vector with the direction and length length of the bone ending at a joint,
          * before any deformations were applied.
          * @return bone vector; zero for the root joint (since it is not the end of a bone)
          */
        Eigen::Vector3d getUndeformedBoneVector(int joint_id);

        /** Get the vector with the direction and length length of the bone ending at a joint
          * @return bone vector; zero for the root joint (since it is not the end of a bone)
          */
        Eigen::Vector3d getBoneVector(int joint_id);

        /** Get the global position of a joint */
        const Eigen::Map<Eigen::Vector3d> & getPosition(int joint_id = JointType::ROOT_PELVIS) const;

        /** Get the local rotation of a bone ending at a joint */
        Eigen::Map<Eigen::Quaterniond> & getLocalRotation(int joint_id);

        /** Get the base rotation of this avatar (all dimensions are rotated by this amount; same as ROOT joint rotation) */
        Eigen::Map<Eigen::Quaterniond> & getCenterRotation();

        /* Set global position of the skeleton's center */
        void setCenterPosition(const Eigen::Vector3d & val);

        /** Set the base rotation of this avatar (everything is rotated by this amount; same as setting ROOT_PELVIS rotation) */
        template<class T> void setCenterRotation(const T & val) { setRotation(JointType::ROOT_PELVIS, val); }

        /** Set the local rotation of the bone ending at a joint to the specified quaternion */
        void setRotation(int joint_id, const Eigen::Quaterniond & quat);

        /** Set the local rotation of the bone ending at a joint to the given AngleAxis object */
        void setRotation(int joint_id, const Eigen::AngleAxisd & angle_axis);

        /** Set the local rotation of the bone ending at a joint to the given euler angles */
        void setRotation(int joint_id, double yaw, double pitch, double roll);

        /** Set the local rotation of the bone ending at a joint so that v1 in the original space rotates to v2 */
        void setRotation(int joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2);

        /** Set the local rotation of the bone ending at a joint so that it points to v */
        void setRotation(int joint_id, const Eigen::Vector3d & v);

        /** Get a pointer to the specified joint */
        Joint::Ptr getJoint(int joint_id) const;

        /** Get joint position in world space */
        Eigen::Vector3d getJointPosition(int joint_id) const;

        /** Get joint position on image */
        Eigen::Vector2d getJointPosition2d(int joint_id) const;

        /** Get the number of joints in the avatar's skeleton */
        int numJoints() const;

        /** Get the weight of the given shape key (blendshape) */
        double & getKeyWeight(int id);

        /** Get the name of the given shape key (blendshape) */
        const std::string & getKeyName(int id) const;

        /** Set the weight of the given shape key (blendshape) */
        void setKeyWeight(int id, double weight);

        /** Get the number of shape keys (blendshapes) available */
        int numKeys() const;

        /** Reset all joints to their initial transforms
          * @param update if true, propagates joint transforms and updates the point cloud.
          *               else, resets joint transforms but does not touch the point cloud
          *               or computed global transforms.
          */
        void reset(bool update = false);

        /** Get the avatar's transformed point cloud
         * @param update if true, updates point cloud according to
         *        the latest transforms before returning
         */
        HumanAvatar::Cloud_T::Ptr getCloud(bool update = false);

        /** Update the avatar point cloud to reflect changes to the joints
         * @param propagate if true, automatically propagates changes throughout the skeleton.
         *       (default is true)
         */
        void update(bool propagate = true);

        /** Color the point cloud by bone weights (this is the default initial coloring when model is loaded) */
        void colorByWeights();

        /** Color the point cloud by groups */
        void color(std::vector<std::vector<int>> & groups);

    private:
        // helper functions, etc. for ceres solver
        /** Update the specified eigen point cloud to reflect changes to the joints
         * @param propagate if true, automatically propagates changes throughout the skeleton.
         *       (default is true)
         */
        template<class T, int opt>
        void _updateCloud(const T * const _w, T * _pt, T * _cache, Eigen::Matrix<T, -1, 3, opt> & out);

        /** Compute the avatar's SMPL pose parameters (Rodrigues angles) */
        template<class T>
        Eigen::Matrix<T, Eigen::Dynamic, 1> _smplParams(const T * const _r) const;

        /** Adds a rotation to the local rotation of the bone ending at a joint */
        void _addRotation(int joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2);

        typedef nanoflann::KDTreeEigenMatrixAdaptor<HumanAvatar::EigenCloud_T, 3, nanoflann::metric_L2_Simple> kd_tree_t;
        typedef std::shared_ptr<kd_tree_t> kd_tree_ptr_t;

        /** Build KD tree index from the provided point cloud */
        kd_tree_ptr_t _buildKDIndex(const EigenCloud_T & data_cloud);

        /** Find nearest neigbors to points in model_cloud within data_cloud, restricted to one NN point for each data_cloud point.
            Outputs to closest, which has size equal to number of points in model_cloud. Each vector in closest (corresponds to model_cloud point)
            is either empty (no matched point) or size 1 (index of matched point in data_cloud) */
        void _findNN(const kd_tree_ptr_t & mindex, const EigenCloud_T & data_cloud, const EigenCloud_T & modelCloud,
            std::vector<std::pair<int, int>> & corres, bool inverted = false);

        /* Ceres-solver cost functor for fitting pose */
        class PoseCostFunctor {
        private:
            HumanAvatar & ava;
            const EigenCloud_T & dataCloud;
            const EigenCloud_T & jointsPrior;
            const cv::Vec4d & intrin;
            GaussianMixture & posePrior;
            std::vector<std::pair<int, int> > & correspondences;

        public:
            /** HYPERPARAMETERS: Weights for different cost function terms */
            double betaJ = 10.0, betaP = 0.2; // betaICP assumed 1

            PoseCostFunctor(HumanAvatar & ava, const EigenCloud_T & data_cloud,
                std::vector<std::pair<int, int>> & correspondences, const EigenCloud_T & joints_prior,
                const cv::Vec4d & pinhole_intrin, GaussianMixture & pose_prior)
                : ava(ava), dataCloud(data_cloud), correspondences(correspondences), jointsPrior(joints_prior),
                  intrin(pinhole_intrin), posePrior(pose_prior) { }

            template <typename T>
            bool operator()(const T* const r, const T* const p, T* residual) const;
        };

        /* Ceres-solver cost functor for fitting shape */
        class ShapeCostFunctor {
        public:
            double betaShape = 0.12, betaJ = 2.0;
            ShapeCostFunctor(HumanAvatar & ava, const EigenCloud_T & data_cloud,
                std::vector<std::pair<int, int>> & correspondences,  const EigenCloud_T & joints_prior,
                const cv::Vec4d & pinhole_intrin)
                : ava(ava), dataCloud(data_cloud), correspondences(correspondences), jointsPrior(joints_prior),
                  intrin(pinhole_intrin) { }

            template <typename T>
            bool operator()(const T* const w, T* residual) const;
        private:
            HumanAvatar & ava;
            const EigenCloud_T & dataCloud;
            const EigenCloud_T & jointsPrior;
            const cv::Vec4d & intrin;
            std::vector<std::pair<int, int>> & correspondences;
        };

        /* Ceres-solver local parameterization adapted for vector of Eigen Quaternions */
        template<int N>
        class MultiQuaternionParameterization : public ceres::LocalParameterization {
        public:
            virtual int GlobalSize() const { return 4*N; }
            virtual int LocalSize() const { return 3*N; }

            bool Plus(const double* x_ptr, const double* delta, double* x_plus_delta_ptr) const;

            bool ComputeJacobian(const double* x, double* jacobian) const;
        };

    public:

        // SECTION Ceres-solver optimization

        /** Fit avatar's pose and shape to the given point cloud */
        void fit(const EigenCloud_T & data_cloud, double deltat = -1.0, bool track = false);
        
        /** Fit avatar's pose and shape to the given point cloud. Please use 'alignToJoints' to initialize before fitting. */
        template<class T>
        void fit(const boost::shared_ptr<pcl::PointCloud<T> > & cloud, double deltat = -1.0, bool track = false);

        /** Fit avatar's pose only. Please use 'alignToJoints' to initialize before fitting. */
        void fitPose(const EigenCloud_T & data_cloud, int max_iter = 8, int num_subiter = 6,
            const std::vector<int> & joint_subset = std::vector<int>(), bool inv_nn = false, kd_tree_ptr_t kd_tree = nullptr);

        /** Fit avatar's shape only. Please use 'alignToJoints' to initialize before fitting. */
        void fitShape(const EigenCloud_T & data_cloud, int max_iter = 8,  int num_subiter = 6, bool inv_nn = false, kd_tree_ptr_t kd_tree = nullptr);

        /** Try to fit avatar's pose parameters, so that joints are approximately aligned to the given positions. Automatically sets joints prior to joint_pos. */
        void alignToJoints(const EigenCloud_T & joint_pos);

        /** Update detected 3d joint positions for this frame */
        void updateJointsPrior(const EigenCloud_T & joint_pos);

        /** Update camera intrinsics */
        void updateCameraIntrin(const cv::Vec4d & intrin);

        /** Visualize the avatar's skeleton and surface in a PCL viewer
         * @param viewer PCL visualizer instance
         * @param pcl_prefix unique prefix to use for PCL id's (so that multiple avatars can be shown at once)
         * @param viewport PCL viewport ID (0 = all)
         */
        void visualize(const pcl::visualization::PCLVisualizer::Ptr & viewer = Visualizer::getPCLVisualizer(),
                       std::string pcl_prefix = "", int viewport = 0) const;

        /** matched joints used for joint prior, given as paired (smpl, mpi) indices */
        static const std::pair<int, int> MATCHED_JOINTS[];

        /** number of matched joints */
        static const int NUM_MATCHED_JOINTS;
    private:

        /** Assign bone/joint weights to each vertex based on distance
          * @param max_vertex_bones maximum number of closest bones to include in weights
          * @param norm_thresh if norm to a bone is above 'norm_threshold' then it is guarenteed to be assigned weight 0
          */
        void assignDistanceWeights(int max_vertex_bones = 4, double norm_thresh = 0.25);

        /** Transform a point in initial world space to a joint's local coordinates,
          * given computed joint positions _pt, spatial transforms _cache */
        template<class T, class VecT_t>
        Eigen::Matrix<T, 3, 1> _toJointSpace(int joint_id, const VecT_t & vec, T * _pt, T * _cache) const;

        /** Transform a point in a joint's local coordinates to world space,
          * given computed joint positions _pt, spatial transforms _cache */
        template<class T, class VecT_t>
        Eigen::Matrix<T, 3, 1> _fromJointSpace(int joint_id, const VecT_t & vec, T * _pt, T * _cache) const;

        /** Transform a point in initial world space to a joint's local coordinates 
          * (using avatar's parameter vectors) */
        Eigen::Vector3d toJointSpace(int joint_id, const Eigen::Vector3d & vec);

        /** propagate local transforms (parameter vectors _r, _s, _p)
          * to global space transforms (_pt, _rt). Assumes joints are topologically sorted */
        template<class T>
        void _propagateJointTransforms(const T * const  _r, const T * const _p, const T * const _w, T * _pb,
                                       T * _pt, T * _rt, T * _cache) const;

        /** propagate local transforms to global space. Assumes joints are topologically sorted */
        void propagateJointTransforms();

        template<class T>
        Eigen::Matrix<T, 3, 1> _computePointPosition(size_t point_index, const T * const _w, T * _pt, T * _cache) const;

        Eigen::Vector3d computePointPosition(size_t point_index);

        // section MODEL PARAMETERS

        /** _w: shape key weights, _r : rotations, _p : global position */
        double * _w, *_r, * _rr, _p[3];

        // internal parameter storage

        // _pb: base position of each joint
        double * _pb;
        // _pt: current position of each joint, _rt: current rotation of each joint (nonessential, may remove in future)
        double *_pt, *_rt;

        // _cache: cached overall transformation from joint's local space to global space 
        double *_cache;
        
        // center position (map to *_p)
        Eigen::Map<Eigen::Vector3d> basePos;

        // section DATA
        // undeformed point cloud (from when model was initially loaded)
        std::unique_ptr<Cloud_T> humanPCBase;

        // deformed point cloud
        Cloud_T::Ptr humanPCTransformed;

        /** stores joints */
        std::vector<Joint::Ptr> joints;
        /** bone weights of each vertex (end joint id, weight); weights should sum to 1 */
        std::vector<std::vector<std::pair<int, double>>> boneWeights;

        /** shape key (blendshape) data */
        std::vector<EigenCloud_T> keyClouds;
        std::vector<std::string> keyNames;

        /** units to increase shape key 0 by to widen the avatar by approximately 1 meter */
        const double PC1_DIST_FACT = 32.0;

        /** joint regressor.
          * the i-th vector contains (index, weight) of points to sum to approx position of joint i */
        std::vector<std::vector<std::pair<int, double>>> jointRegressor;

        /** joint position prior (detections derived from 2D pose CNN) */
        EigenCloud_T jointsPrior;

        /** pinhole camera intrinsics for computing joint prior: (fx, cx, fy, cy) */
        cv::Vec4d pinholeIntrin = cv::Vec4d(426.55529223, 423.28048382, 426.55654991, 245.95310243);

public: // debug: public to allow displaying pose prior in PCL viewer

        /** pose prior (a Gaussian Mixture Model) */
        GaussianMixture posePrior;
private:

        friend struct Joint;
        friend struct HumanAvatarUKFModel;
    };
}
