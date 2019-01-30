#pragma once
#include "Version.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>

namespace ark {
    // TODO: refactor this file
    /** Gaussian Mixture Model */
    class GaussianMixture {        
    public:
        /** load Gaussian Mixture parameters from 'path' */
        void load(const std::string & path);

        /** get number of Gaussian mixture components */
        inline int numComponents() const { return nComps; };

        template<class T>
        /** Compute PDF at 'input' */
        T pdf(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x) const {
            T prob = 0.0;
            typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_t;
            for (int i = 0; i < nComps; ++i) {
                Eigen::TriangularView<Eigen::MatrixXd, Eigen::Lower> L(cov_cho[i]);
                auto residual = (L.transpose() * (x - mean.row(i).transpose()));
                prob += consts[i] * ceres::exp(-0.5 * residual.squaredNorm());
            }
            return prob;
        }

        template<class T>
        /** Compute Ceres residual vector (squaredNorm of output vector is equal to min_i -log(c_i pdf_i(x))) */
        Eigen::Matrix<T, Eigen::Dynamic, 1> residual(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x) {
            T bestProb = T(0);
            typedef Eigen::Matrix<T, Eigen::Dynamic, 1> vec_t;
            typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_t;
            vec_t ans;
            for (int i = 0; i < nComps; ++i) {
                Eigen::TriangularView<Eigen::MatrixXd, Eigen::Lower> L(cov_cho[i]);
                vec_t residual(nDims + 1);
                residual[nDims] = T(0);
                residual.head(nDims) = L.transpose() * (x - mean.row(i).transpose()) * sqrt(0.5);
                T p = residual.squaredNorm() - T(consts_log[i]);
                if (p < bestProb || !i) {
                    bestProb = p;
                    residual[nDims] = T(sqrt(-consts_log[i]));
                    ans = residual;
                }
            }
            return ans;
        }
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

        /** Names for the various skeletal joints in the SMPL model (does not work for other models) */
        typedef enum _JointType {
            // TODO: delegate to a skeleton instead

            // BFS Order (topologically sorted)
            ROOT = 0, PELVIS = 0, L_HIP, R_HIP, SPINE1, L_KNEE, R_KNEE, SPINE2, L_ANKLE,
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
        double * w() { return _w; }

        /** Get the avatar's bone rotation parameter vector (stores quaternions: x y z w)
          * (first 4 indices are root rotation quaternion, total size is numJoints() * 4) */
        double * r() { return _r; }

        /** Get the avatar's global position (length 3) */
        double * p() { return _p; }

        /** Compute the avatar's SMPL pose parameters (Rodrigues angles) */
        Eigen::VectorXd smplParams() const;

        // utility functions for modifying the pose

        /* Get global position of the skeleton's center */
        Eigen::Map<Eigen::Vector3d> getBasePosition();

        /** Get the vector with the direction and length length of the bone ending at a joint,
          * before any deformations were applied.
          * @return bone vector; zero for the root joint (since it is not the end of a bone)
          */
        Eigen::Vector3d getUndeformedBoneVector(JointType joint_id);

        /** Get the vector with the direction and length length of the bone ending at a joint
          * @return bone vector; zero for the root joint (since it is not the end of a bone)
          */
        Eigen::Vector3d getBoneVector(JointType joint_id);

        /** Get the global position of a joint */
        const Eigen::Map<Eigen::Vector3d> & getPosition(JointType joint_id = JointType::PELVIS) const;

        /** Get the local rotation of a bone ending at a joint */
        Eigen::Map<Eigen::Quaterniond> & getLocalRotation(JointType joint_id);

        /** Get the base rotation of this avatar (all dimensions are rotated by this amount; same as ROOT joint rotation) */
        Eigen::Map<Eigen::Quaterniond> & getCenterRotation();

        /* Set global position of the skeleton's center */
        void setCenterPosition(const Eigen::Vector3d & val);

        /** Set the base rotation of this avatar (everything is rotated by this amount; same as setting ROOT/PELVIS rotation) */
        template<class T> void setCenterRotation(const T & val) { setRotation(JointType::ROOT, val); }

        /** Set the local rotation of the bone ending at a joint to the specified quaternion */
        void setRotation(JointType joint_id, const Eigen::Quaterniond & quat);

        /** Set the local rotation of the bone ending at a joint to the given AngleAxis object */
        void setRotation(JointType joint_id, const Eigen::AngleAxisd & angle_axis);

        /** Set the local rotation of the bone ending at a joint to the given euler angles */
        void setRotation(JointType joint_id, double yaw, double pitch, double roll);

        /** Set the local rotation of the bone ending at a joint so that v1 in the original space rotates to v2 */
        void setRotation(JointType joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2);

        /** Set the local rotation of the bone ending at a joint so that it points to v */
        void setRotation(JointType joint_id, const Eigen::Vector3d & v);

        /** Get a pointer to the specified joint */
        Joint::Ptr getJoint(JointType joint_id) const;

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
        void _updateCloud(const T * const _w, T * _pt, T * _cache, Eigen::Matrix<T, -1, 3, opt> & out) {
            for (size_t i = 0; i < out.rows(); ++i) {
                out.row(i) = _computePointPosition(i, _w, _pt, _cache).template cast<T>();
            }
        }

        /** Compute the avatar's SMPL pose parameters (Rodrigues angles) */
        template<class T>
        Eigen::Matrix<T, Eigen::Dynamic, 1> _smplParams(const T * const _r) const {
            Eigen::Matrix<T, Eigen::Dynamic, 1> res;
            res.resize((NUM_JOINTS - 1) * 3);
            for (int i = 1; i < NUM_JOINTS; ++i) {
                const Eigen::Map<const Eigen::Quaternion<T>> q(_r + i * NUM_ROT_PARAMS);

                T n = q.vec().norm();
                if (n == T(0)) {
                    res.segment<3>((i - 1) * 3).setZero();
                }
                else {
                    if (q.w() < T(0)) n = -n;
                    res.segment<3>((i - 1) * 3) = q.vec() / n * T(2) * ceres::atan2(n, abs(q.w()));
                }
            }
            return res;
        }

        /** Adds a rotation to the local rotation of the bone ending at a joint */
        void _addRotation(JointType joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2);

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
        public:
            /** HYPERPARAMETERS: Weights for different cost function terms */
            double betaP = 3.0, betaS = 0.55; // betaICP assumed 1

            PoseCostFunctor(HumanAvatar & ava, const EigenCloud_T & data_cloud,
                std::vector<std::pair<int, int>> & correspondences, const EigenCloud_T & joints_prior, GaussianMixture & pose_prior)
                : ava(ava), dataCloud(data_cloud), correspondences(correspondences), jointsPrior(joints_prior), posePrior(pose_prior) { }

            template <typename T>
            bool operator()(const T* const r, const T* const p, T* residual) const {
                const int NUM_JOINTS = HumanAvatar::JointType::_COUNT;
                /*
                // to fix parameters at initial value (debug)
                T w[NUM_SHAPEKEYS];
                for (int i = 0; i < NUM_SHAPEKEYS; ++i) w[i] = T(ava.w()[i]);
                T r[NUM_JOINTS * NUM_ROT_PARAMS];
                for (int i = 0; i < NUM_JOINTS * NUM_ROT_PARAMS; ++i) r[i] = T(ava.r()[i]);
                for (int i = 0; i < NUM_JOINTS * NUM_SCALE_PARAMS; ++i) s[i] = T(ava.s()[i]);
                */
                T w[NUM_SHAPEKEYS];
                for (int i = 0; i < NUM_SHAPEKEYS; ++i) w[i] = T(ava.w()[i]);

                T pb[NUM_JOINTS * NUM_POS_PARAMS], pt[NUM_JOINTS * NUM_POS_PARAMS],
                  rt[NUM_JOINTS * NUM_ROT_PARAMS], cache[NUM_JOINTS * NUM_ROT_MAT_PARAMS];
                // propagate skeletal transforms
                ava._propagateJointTransforms(r, p, w, pb, pt, rt, cache);

                typedef Eigen::Matrix<T, 3, 1> Vec3T;
                typedef Eigen::Matrix<T, 2, 1> Vec2T;
                typedef Eigen::Map<Eigen::Matrix<T, 3, 1> > Vec3TMap;
                typedef Eigen::Map<Eigen::Matrix<T, 2, 1> > Vec2TMap;

                T * residualPtr = residual;
                for (auto & cor : correspondences) {
                    // compute position of each point using transformed data
                    Vec3T surfPoint = ava._computePointPosition(cor.first, w, pt, cache);
                    Vec3TMap res(residualPtr);
                    res = surfPoint - dataCloud.row(cor.second).transpose();
                    residualPtr += NUM_POS_PARAMS;
                }

                for (int i = 0; i < NUM_JOINTS; ++i) {
                    Vec2TMap res(residualPtr);
                    if (std::isnan(jointsPrior.row(i).x())) {
                        res = Vec2T::Zero();
                    } else {
                        Vec3TMap currjointPosition(pt + i * NUM_POS_PARAMS);
                        res = (currjointPosition.head<2>() - jointsPrior.row(i).transpose().head<2>()) * betaP;
                    }
                    residualPtr += 2;
                }

                Eigen::Map<Eigen::Matrix<T, NUM_JOINTS * 3 - 2, 1>> posePriorResidual(residualPtr);
                posePriorResidual = posePrior.residual(ava._smplParams(r)) * betaS;
                return true;
            }
        private:
            HumanAvatar & ava;
            const EigenCloud_T & dataCloud, & jointsPrior;
            GaussianMixture & posePrior;
            std::vector<std::pair<int, int> > & correspondences;
        };

        /* Ceres-solver cost functor for fitting shape */
        class ShapeCostFunctor {
        public:
            double betaShape = 0.1;
            ShapeCostFunctor(HumanAvatar & ava, const EigenCloud_T & data_cloud,
                std::vector<std::pair<int, int>> & correspondences)
                : ava(ava), dataCloud(data_cloud), correspondences(correspondences) { }

            template <typename T>
            bool operator()(const T* const w, T* residual) const {
                const int NUM_JOINTS = HumanAvatar::JointType::_COUNT;
                // to fix parameters at initial value 
                T r[NUM_JOINTS * NUM_ROT_PARAMS], p[NUM_POS_PARAMS];
                for (int i = 0; i < NUM_JOINTS * NUM_ROT_PARAMS; ++i) r[i] = T(ava.r()[i]);
                for (int i = 0; i < NUM_POS_PARAMS; ++i) p[i] = T(ava.p()[i]);

                T pb[NUM_JOINTS * NUM_POS_PARAMS], pt[NUM_JOINTS * NUM_POS_PARAMS], rt[NUM_JOINTS * NUM_ROT_PARAMS], cache[NUM_JOINTS * NUM_ROT_MAT_PARAMS];
                // simulate propagation of skeletal transforms
                ava._propagateJointTransforms(r, p, w, pb, pt, rt, cache);

                typedef Eigen::Matrix<T, 3, 1> Vec3T;
                typedef Eigen::Map<Vec3T> Vec3TMap;

                T * residualPtr = residual;
                for (auto & cor : correspondences) {
                    // compute position of each point using transformed data
                    Vec3T surfPoint = ava._computePointPosition(cor.first, w, pt, cache);
                    Vec3TMap res(residualPtr);
                    res = surfPoint - dataCloud.row(cor.second).transpose();
                    residualPtr += NUM_POS_PARAMS;
                }
                for (int i = 0; i < NUM_SHAPEKEYS; ++i) {
                    residualPtr[i] = w[i] * betaShape;
                }

                return true;
            }
        private:
            HumanAvatar & ava;
            const EigenCloud_T & dataCloud;
            std::vector<std::pair<int, int>> & correspondences;
        };

        /* Ceres-solver local parameterization adapted for vector of Eigen Quaternions */
        template<int N>
        class MultiQuaternionParameterization : public ceres::LocalParameterization {
        public:
            virtual int GlobalSize() const { return 4*N; }
            virtual int LocalSize() const { return 3*N; }

            bool Plus(const double* x_ptr, const double* delta, double* x_plus_delta_ptr) const {
                for (int i = 0; i < N; ++i) {
                    Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_ptr + (i<<2));
                    Eigen::Map<const Eigen::Quaterniond> x(x_ptr + (i<<2));
                    const double * delta_ = delta + 3*i;
                    const double norm_delta =
                        sqrt(delta_[0] * delta_[0] + delta_[1] * delta_[1] + delta_[2] * delta_[2]);
                    if (norm_delta > 0.0) {
                        const double sin_delta_by_delta = sin(norm_delta) / norm_delta;
                        Eigen::Quaterniond delta_q(cos(norm_delta),
                            sin_delta_by_delta * delta_[0],
                            sin_delta_by_delta * delta_[1],
                            sin_delta_by_delta * delta_[2]);
                        x_plus_delta = delta_q * x;
                    }
                    else x_plus_delta = x;
                }
                return true;
            }

            bool ComputeJacobian(const double* x, double* jacobian) const {
                memset(jacobian, 0, 4 * 3 * N*N * sizeof(jacobian[0]));
                double * jacobian_ = jacobian;
                for (int i = 0; i < N; ++i) {
                    const double * x_ = x + (i<<2);
                    jacobian_[0] = x_[3]; jacobian_[1] = x_[2]; jacobian_[2] = -x_[1];
                    jacobian_ += 3 * N;
                    jacobian_[0] = -x_[2]; jacobian_[1] = x_[3]; jacobian_[2] = x_[0];
                    jacobian_ += 3 * N;
                    jacobian_[0] = x_[1]; jacobian_[1] = -x_[0]; jacobian_[2] = x_[3];
                    jacobian_ += 3 * N;
                    jacobian_[0] = -x_[0]; jacobian_[1] = -x_[1]; jacobian_[2] = -x_[2];
                    jacobian_ += 3 * N + 3;
                }
                return true;
            }
        };

    public:

        // SECTION Ceres-solver optimization

        /** Fit avatar's pose and shape to the given point cloud */
        void fit(const EigenCloud_T & data_cloud);

		void fitTrack(const EigenCloud_T & data_cloud);

        /** Fit avatar's pose and shape to the given point cloud. Please use 'alignToJoints' to initialize before fitting. */
        template<class T>
        void fit(const boost::shared_ptr<pcl::PointCloud<T> > & cloud) {
            // store point cloud in Eigen format
            EigenCloud_T dataCloud(cloud->points.size(), 3);
            for (size_t i = 0; i < cloud->points.size(); ++i) {
                dataCloud.row(i) = cloud->points[i].getVector3fMap().cast<double>();
            }
            fit(dataCloud);
        }

		template<class T>
        /** Track avatar's pose and shape given a new point cloud. Please use 'setJointsPrior' to update the joints prior before calling this. */
		void fitTrack(const boost::shared_ptr<pcl::PointCloud<T> > & cloud) {
			// store point cloud in Eigen format
			EigenCloud_T dataCloud(cloud->points.size(), 3);
			for (size_t i = 0; i < cloud->points.size(); ++i) {
				dataCloud.row(i) = cloud->points[i].getVector3fMap().cast<double>();
			}
			fitTrack(dataCloud);
		}

        /** Fit avatar's pose only. Please use 'alignToJoints' to initialize before fitting. */
        void fitPose(const EigenCloud_T & data_cloud, int max_iter = 8, int num_subiter = 6,
            const std::vector<int> & joint_subset = std::vector<int>(), bool inv_nn = false, kd_tree_ptr_t kd_tree = nullptr);

        /** Fit avatar's shape only. Please use 'alignToJoints' to initialize before fitting. */
        void fitShape(const EigenCloud_T & data_cloud, int max_iter = 8,  int num_subiter = 6, bool inv_nn = false, kd_tree_ptr_t kd_tree = nullptr);

        /** Try to fit avatar's pose parameters, so that joints are approximately aligned to the given positions. Automatically sets joints prior to joint_pos. */
        void alignToJoints(const EigenCloud_T & joint_pos);

        /** Update detected joint positions for this frame (only x, y positions of joints will be used to compute the error) */
        void updateJointsPrior(const EigenCloud_T & joint_pos);

        /** Visualize the avatar's skeleton and surface in a PCL viewer
         * @param viewer PCL visualizer instance
         * @param pcl_prefix unique prefix to use for PCL id's (so that multiple avatars can be shown at once)
         * @param viewport PCL viewport ID (0 = all)
         */
        void visualize(pcl::visualization::PCLVisualizer::Ptr & viewer = Visualizer::getPCLVisualizer(),
                       std::string pcl_prefix = "", int viewport = 0) const;
    private:

        /** Assign bone/joint weights to each vertex based on distance
          * @param max_vertex_bones maximum number of closest bones to include in weights
          * @param norm_thresh if norm to a bone is above 'norm_threshold' then it is guarenteed to be assigned weight 0
          */
        void assignDistanceWeights(int max_vertex_bones = 4, double norm_thresh = 0.25);

        /** Transform a point in initial posture space to a joint's transformed posture space,
          * given computer position, transform vectors _pt, _cache */
        template<class T, class VecT_t>
        Eigen::Matrix<T, 3, 1> inline _toJointSpace(JointType joint_id, const VecT_t & vec, T * _pt, T * _cache) {
            return Eigen::Map<Eigen::Matrix<T, 3, 3> >(_cache + joint_id * NUM_ROT_MAT_PARAMS) * vec
                 + Eigen::Map<const Eigen::Matrix<T, 3, 1>> (_pt + joint_id * NUM_POS_PARAMS);
        }

        /** Transform a point in initial posture space to a joint's transformed posture space
          * (using avatar's parameter vectors) */
        Eigen::Vector3d toJointSpace(JointType joint_id, const Eigen::Vector3d & vec);

        /** propagate local transforms (parameter vectors _r, _s, _p)
          * to global space transforms (_pt, _rt). Assumes joints are topologically sorted */
        template<class T>
        void _propagateJointTransforms(const T * const  _r, const T * const _p, const T * const _w, T * _pb,
                                       T * _pt, T * _rt, T * _cache) {
            typedef Eigen::Map<const Eigen::Matrix<T, 2, 1>> const_vec2map;
            typedef Eigen::Matrix<T, 3, 1> vec_t;
            typedef Eigen::Map<vec_t> vecmap;
            typedef Eigen::Map<const vec_t> const_vecmap;
            typedef Eigen::Map<Eigen::Quaternion<T>> quatmap;
            typedef Eigen::Map<const Eigen::Quaternion<T>> const_quatmap;
            memcpy(_rt, _r, NUM_ROT_PARAMS * sizeof(T));
            memcpy(_pt, _p, NUM_POS_PARAMS * sizeof(T));

            const_quatmap rotCen(_r);
            Eigen::Map<Eigen::Matrix<T, 3, 3> > ca(_cache);
            ca = rotCen.toRotationMatrix();

            for (size_t jid = 0; jid < joints.size(); ++jid) {
                vecmap basePos(_pb + jid * NUM_POS_PARAMS);
                //basePos = joints[jid]->posBase.cast<T>();

                basePos = vec_t(T(0), T(0), T(0));
                for (auto regrEntry : jointRegressor[jid]) {
                    vec_t blend = humanPCBase->points[regrEntry.first].getVector3fMap().cast<T>();
                    for (size_t j = 0; j < keyNames.size(); ++j) {
                        blend += keyClouds[j].row(regrEntry.first) * _w[j];
                    }
                    basePos += blend * regrEntry.second;
                }
            }

            for (size_t jid = 1; jid < joints.size(); ++jid) {
                Joint::Ptr & joint = joints[jid];
                JointType parID = joint->parent->type;

                const size_t ROT_IDX = jid * NUM_ROT_PARAMS, POS_IDX = jid * NUM_POS_PARAMS;
                quatmap rotTransformed(_rt + ROT_IDX);
                rotTransformed = quatmap(_rt + parID * NUM_ROT_PARAMS) * const_quatmap(_r + ROT_IDX);
                
                Eigen::Map<Eigen::Matrix<T, 3, 3> >(_cache + NUM_ROT_MAT_PARAMS * jid) = rotTransformed.toRotationMatrix();

                vecmap(_pt + POS_IDX) = _toJointSpace(joint->parent->type,
                             vecmap(_pb + POS_IDX) - vecmap(_pb + parID * NUM_POS_PARAMS), _pt, _cache);
            }
        }

        /** propagate local transforms to global space. Assumes joints are topologically sorted */
        void propagateJointTransforms();

        template<class T>
        inline Eigen::Matrix<T, 3, 1> _computePointPosition(size_t point_index, const T * const _w, T * _pt, T * _cache) {
            const Point_T & initPt_PCL = humanPCBase->points[point_index];
            typedef Eigen::Matrix<T, 3, 1> vec_t;
            vec_t result = vec_t::Zero(), blend = initPt_PCL.getVector3fMap().cast<T>();

            // add blendshapes
            for (size_t j = 0; j < keyNames.size(); ++j) {
                blend += keyClouds[j].row(point_index) * _w[j];
            }

            // linear blend skinning (LBS)
            for (const auto & bw : boneWeights[point_index]) {
                vec_t initPt = blend - joints[bw.first]->posBase;
                result += _toJointSpace(static_cast<JointType>(bw.first), initPt, _pt, _cache) * T(bw.second);
            }
            return result;
        }

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
        boost::unique_ptr<Cloud_T> humanPCBase;

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

        /** joint position prior (detections derived from 2D pose CNN;
          * only x, y will be used for error term since computed depth can be inaccurate) */
        EigenCloud_T jointsPrior;

public: // debug: public to allow displaying pose prior in PCL viewer

        /** pose prior (a Gaussian Mixture Model) */
        GaussianMixture posePrior;
private:

        friend struct Joint;
    };
}
