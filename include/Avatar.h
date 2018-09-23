#pragma once
#include "Version.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ark {

    /** Represents a humanoid avatar */
    class HumanAvatar {
    public:
        typedef pcl::PointXYZRGBA Point_T;
        typedef pcl::PointCloud<Point_T> Cloud_T;
        typedef Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> EigenCloud_T;

        /** Names for the various skeletal joints in the SMPL model (does not work for other models) */
        typedef enum _JointType {
            // TODO: delegate to a skeleton instead

            // DFS Order
            // ROOT = 0, PELVIS = 0, L_HIP, L_KNEE, L_ANKLE, L_FOOT, R_HIP, R_KNEE, R_ANKLE,
            // R_FOOT, SPINE1, SPINE2, SPINE3, NECK, HEAD, L_COLLAR, L_SHOULDER,
            // L_ELBOW, L_WRIST, L_HAND, R_COLLAR, R_SHOULDER, R_ELBOW, R_WRIST, R_HAND

            // BFS Order
            ROOT = 0, PELVIS = 0, R_HIP, L_HIP, SPINE1, R_KNEE, L_KNEE, SPINE2, R_ANKLE,
            L_ANKLE, SPINE3, R_FOOT, L_FOOT, NECK, R_COLLAR, L_COLLAR, HEAD, R_SHOULDER,
            L_SHOULDER, R_ELBOW, L_ELBOW, R_WRIST, L_WRIST, R_HAND, L_HAND,

            _COUNT
        } JointType;

        /** Represents a joint in the model's skeleton. Contains rotation, scale information about bone ending at the joint.
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

            Eigen::Map<Eigen::Vector3d> posBase,     // (defined) position of joint from skeleton file
                posTransformed,                      // (computed) current global position of joint
                scale;                               // (PARAMETER) absolute scale of bone along local axes (x is along bone), set bhy user
            Eigen::Map<Eigen::Quaterniond> rotBase,  // (computed) rotation from bone ending at joint
                                                     //            to the positive x-axis in initial position
                rotTransformed,                      // (computed) current global rotation of bone ending at joint
                rotation;                            // (PARAMETER) local rotation of bone ending at joint, set by user

            /** Contruct a joint associated with the given avatar & joint type */
            Joint(HumanAvatar & avatar, JointType type);
        };

        /** The directory the avatar's model was imported from */
        const std::string MODEL_DIR;

        /** Create an avatar from the model information in 'model_dir'
         *  @param model_dir path to directory containing model files
         */
        explicit HumanAvatar(const std::string & model_dir);

        /** Create an avatar from the model information in 'model_dir'
         *  @param model_dir path to directory containing model files
         *  @param shape_keys names of shape keys in 'model_dir'/shapekey to use
         */
        HumanAvatar(const std::string & model_dir, const std::vector<std::string> & shape_keys);

        /** Destructor for HumanAvatar */
        ~HumanAvatar();

        // section GETTERS/SETTERS

        // model parameter vector getters/setters
        /** Get the avatar's key weight parameter vector
          * (ith index is relative weight of ith shape key a.k.a. blendshape;
          *  total size is numKeys()) */
        double * w() { return _w; }

        /** Get the avatar's bone scale parameter vector (stores 3D vectors: x y z)
          * (first 3 indices are root scale, total size is numJoints() * 3) */
        double * s() { return _s; }

        /** Get the avatar's bone rotation parameter vector (stores quaternions: x y z w)
          * (first 4 indices are root rotation quaternion, total size is numJoints() * 4) */
        double * r() { return _r; }

        /** Get the avatar's global position (length 3) */
        double * p() { return _p; }

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

        /** Get the local scale of a bone ending at a joint (x-axis is bone direction) */
        Eigen::Map<Eigen::Vector3d> & getLocalScale(JointType joint_id);

        /** Get the base rotation of this avatar (all dimensions are scaled by this amount; same as ROOT joint rotation) */
        Eigen::Map<Eigen::Quaterniond> & getBaseRotation();

        /** Get the base scale of this avatar (all dimensions are scaled by this amount; same as ROOT joint scale) */
        Eigen::Map<Eigen::Vector3d> & getBaseScale();

        /* Set global position of the skeleton's center */
        void setCenterPosition(const Eigen::Vector3d & val);

        /** Set the base rotation of this avatar (everything is rotated by this amount; same as setting ROOT/PELVIS rotation) */
        template<class T> void setCenterRotation(const T & val) { setRotation(JointType::ROOT, val); }

        /** Set the base scale of this avatar (all dimensions are scaled by this amount; same as setting ROOT/PELVIS scale) */
        void setCenterScale(const Eigen::Vector3d & val);

        /** Set the local rotation of the bone ending at a joint to the specified quaternion */
        void setRotation(JointType joint_id, const Eigen::Quaterniond & quat);

        /** Set the local rotation of the bone ending at a joint to the given AngleAxis object */
        void setRotation(JointType joint_id, const Eigen::AngleAxisd & angle_axis);

        /** Set the local rotation of the bone ending at a joint to the given euler angles */
        void setRotation(JointType joint_id, double yaw, double pitch, double roll);

        /** Set the local rotation of the bone ending at a joint so that v1 in the original space rotates to v2 */
        void setRotation(JointType joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2);

    private:
        /** Adds a rotation to the local rotation of the bone ending at a joint */
        void _addRotation(JointType joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2);
    public:

        /** Set the local rotation of the bone ending at a joint so that it points to v */
        void setRotation(JointType joint_id, const Eigen::Vector3d & v);


        /** Set the local rotation of the bone ending at a joint so that
          * it points to AND has the length of v */
        void setBoneVector(JointType joint_id, const Eigen::Vector3d & v);

        /** Set the local scale of the bone ending at a joint */
        void setScale(JointType joint_id, const Eigen::Vector3d & scale);

        /** Set the local scale of the bone ending at a joint */
        void setScale(JointType joint_id, double x, double y, double z);

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
        void _updateCloud(const T * const _w, T * _pt, T * _rt, const T * const _s,
            Eigen::Matrix<T, -1, 3, opt> & out) {
            for (size_t i = 0; i < out.rows(); ++i) {
                out.row(i) = _computePointPosition(i, _w, _pt, _rt, _s).cast<T>();
            }
        }

        /** Find nearest neigbors to points in dataCloud within modelCloud. Outputs to closest, which has a vector
            for each point in modelCloud, containing indices of points that are closest to the current model point */
        void _findNN(const EigenCloud_T & dataCloud, const EigenCloud_T & modelCloud,
            std::vector<std::vector<int>> & closest);

        /* Ceres-solver cost functor for alignment ICP */
        class AlignICPCostFunctor {
        public:
            AlignICPCostFunctor(HumanAvatar & ava, EigenCloud_T & dataCloud,
                std::vector<std::vector<int>> & closest)
                : ava(ava), dataCloud(dataCloud), closest(closest) { }

            template <typename T>
            bool operator()(const T* const _r, const T* const _p, T* residual) const {
                const int NUM_JOINTS = HumanAvatar::JointType::_COUNT;
                T w[NUM_SHAPEKEYS], r[NUM_JOINTS * NUM_ROT_PARAMS], s[NUM_JOINTS * NUM_SCALE_PARAMS];
                for (int i = 0; i < NUM_SHAPEKEYS; ++i) w[i] = T(ava.w()[i]);
                for (int i = NUM_ROT_PARAMS; i < NUM_JOINTS * NUM_ROT_PARAMS; ++i) r[i] = T(ava.r()[i]);
                for (int i = 0; i < NUM_JOINTS * NUM_SCALE_PARAMS; ++i) s[i] = T(ava.s()[i]);

                for (int i = 0; i < NUM_ROT_PARAMS; ++i) r[i] = _r[i];

                T pt[NUM_JOINTS * NUM_POS_PARAMS], rt[NUM_JOINTS * NUM_ROT_PARAMS];
                // simulate propagation of skeletal transforms
                ava._propagateJointTransforms(r, s, _p, pt, rt);
                residual[0] = T(0);
                for (size_t i = 0; i < ava.getCloud(false)->size(); ++i) {
                    if (closest[i].empty()) continue; // short-circuit if no closest points
                    Eigen::Matrix<T, 3, 1> point = ava._computePointPosition(i, w, pt, rt, s);
                    for (int j : closest[i]) {
                        // compute position of each point using transformed data
                        residual[0] += (point - dataCloud.row(j).transpose().cast<T>()).squaredNorm();
                    }
                }
                return true;
            }
        private:
            HumanAvatar & ava;
            EigenCloud_T & dataCloud;
            std::vector<std::vector<int>> & closest;
        };

        /* Ceres-solver cost functor for fitting */
        class AvatarFitCostFunctor {
        public:
            AvatarFitCostFunctor(HumanAvatar & ava, EigenCloud_T & dataCloud,
                std::vector<std::vector<int>> & closest)
                : ava(ava), dataCloud(dataCloud), closest(closest) { }

            template <typename T>
            bool operator()(const T* const w, const T* const r, const T* const s, const T* const p, T* residual) const {
                const int NUM_JOINTS = HumanAvatar::JointType::_COUNT;
                /*
                // to fix parameters (debug)
                T w[NUM_SHAPEKEYS], r[NUM_JOINTS * NUM_ROT_PARAMS], s[NUM_JOINTS * NUM_SCALE_PARAMS];
                for (int i = 0; i < NUM_SHAPEKEYS; ++i) w[i] = T(ava.w()[i]);
                for (int i = 0; i < NUM_JOINTS * NUM_ROT_PARAMS; ++i) r[i] = T(ava.r()[i]);
                for (int i = 0; i < NUM_JOINTS * NUM_SCALE_PARAMS; ++i) s[i] = T(ava.s()[i]);
                */

                T pt[NUM_JOINTS * NUM_POS_PARAMS], rt[NUM_JOINTS * NUM_ROT_PARAMS];
                // simulate propagation of skeletal transforms
                ava._propagateJointTransforms(r, s, p, pt, rt);
                residual[0] = T(0);
                for (size_t i = 0; i < ava.getCloud(false)->size(); ++i) {
                    if (closest[i].empty()) continue; // short-circuit if no closest points
                    Eigen::Matrix<T, 3, 1> point = ava._computePointPosition(i, w, pt, rt, s);
                    for (int j : closest[i]) {
                        // compute position of each point using transformed data
                        residual[0] += (point - dataCloud.row(j).transpose().cast<T>()).squaredNorm();
                    }
                }
                return true;
            }
        private:
            HumanAvatar & ava;
            EigenCloud_T & dataCloud;
            std::vector<std::vector<int>> & closest;
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
        /** Align center position, rotation, scale to cloud for the given number of iterations */
        void alignICP(const Cloud_T::Ptr & cloud, int n_iter = 10);

    private:
        /** Align center position, rotation, scale to cloud for the given number of iterations */
        void _alignICP(EigenCloud_T & dataCloud, EigenCloud_T & modelCloud, int n_iter = 10);
    public:

        /** Fit avatar's pose and shape to the given point cloud */
        void fit(const Cloud_T::Ptr & cloud);

        /** Visualize the avatar's skeleton in a PCL viewer
         * @param viewer PCL visualizer instance
         * @param viewport PCL viewport ID
         * @param pcl_prefix prefix to use for PCL id's (so that multiple skeletons can be shown at once)
         */
        void visualize(pcl::visualization::PCLVisualizer::Ptr & viewer, std::string pcl_prefix = "", int viewport = 0) const;

        // sizes of data types (in bytes) for each parameter vector type
        static const int NUM_ROT_PARAMS = 4, NUM_SCALE_PARAMS = 3, NUM_POS_PARAMS = 3;
        // weight key, joint numbers. This allows us to use static cost function with Ceres.
        static const int NUM_SHAPEKEYS = 10, NUM_JOINTS = HumanAvatar::JointType::_COUNT;
    private:

        /** Assign bone/joint weights to each vertex based on distance
          * @param max_vertex_bones maximum number of closest bones to include in weights
          * @param norm_thresh if norm to a bone is above 'norm_threshold' then it is guarenteed to be assigned weight 0
          */
        void assignDistanceWeights(int max_vertex_bones = 4, double norm_thresh = 0.25);

        /** Transform a point in initial posture space to a joint's transformed posture space,
          * given parameter vectors _pt, _rt */
        template<class T, class VecT_t>
        Eigen::Matrix<T, 3, 1> inline _toJointSpace(JointType joint_id,
            const VecT_t & vec, T * _pt, T * _rt, const T * const _s) {
            Joint::Ptr & joint = joints[joint_id];
            Joint * parent = joint->parent;
            if (parent == nullptr) return vec + joint->posTransformed.cast<T>();
            Eigen::Matrix<T, 3, 1> v = vec;

            JointType parID = parent->type;

            typedef Eigen::Map<const Eigen::Matrix<T, 3, 1>> vecmap;
            typedef Eigen::Map<Eigen::Quaternion<T>> quatmap;

            v -= parent->posBase.cast<T>();
            const auto & rotBaseO = joint->rotBase;
            Eigen::Quaternion<T> rotBase(T(rotBaseO.w()), T(rotBaseO.x()), T(rotBaseO.y()), T(rotBaseO.z()));
            v = util::rotate(v, rotBase.inverse());
            v = v.cwiseProduct(vecmap(_s + joint_id * NUM_SCALE_PARAMS));
            v = util::rotate(v, rotBase);
            v = util::rotate(v, quatmap(_rt + joint_id * NUM_ROT_PARAMS));
            v = v.cwiseProduct(vecmap(_s));
            v += vecmap(_pt + parID * NUM_POS_PARAMS);

            return v;
        }

        /** Transform a point in initial posture space to a joint's transformed posture space
          * (using avatar's parameter vectors) */
        Eigen::Vector3d toJointSpace(JointType joint_id, const Eigen::Vector3d & vec);

        /** propagate local transforms (parameter vectors _r, _s, _p)
          * to global space transforms (_pt, _rt). Assumes joints are topologically sorted */
        template<class T>
        void _propagateJointTransforms(const T * const  _r, const T * const _s, const T * const _p, T * _pt, T * _rt) {
            typedef Eigen::Map<Eigen::Matrix<T, 3, 1>> vecmap;
            typedef Eigen::Map<Eigen::Quaternion<T>> quatmap;
            typedef Eigen::Map<const Eigen::Quaternion<T>> const_quatmap;
            memcpy(_rt, _r, NUM_ROT_PARAMS * sizeof(_r[0]));
            memcpy(_pt, _p, NUM_POS_PARAMS * sizeof(_p[0]));
            for (size_t jid = 1; jid < joints.size(); ++jid) {
                Joint::Ptr & joint = joints[jid];
                JointType parID = joint->parent->type;

                const size_t ROT_IDX = jid * NUM_ROT_PARAMS, POS_IDX = jid * NUM_POS_PARAMS;
                quatmap(_rt + ROT_IDX) = quatmap(_rt + parID * NUM_ROT_PARAMS) * const_quatmap(_r + ROT_IDX);
                vecmap(_pt + POS_IDX) = _toJointSpace(static_cast<JointType>(jid),
                    joint->posBase.cast<T>(), _pt, _rt, _s);
            }
        }

        /** propagate local transforms to global space. Assumes joints are topologically sorted */
        void propagateJointTransforms();

        template<class T>
        inline Eigen::Matrix<T, 3, 1> _computePointPosition(size_t point_index, const T * const _w, T * _pt, T * _rt, const T * const _s) {
            const Point_T & initPt_PCL = humanPCBase->points[point_index];
            typedef Eigen::Matrix<T, 3, 1> vec_t;
            vec_t result(T(0), T(0), T(0)), initPt(T(initPt_PCL.x), T(initPt_PCL.y), T(initPt_PCL.z));

            // add blendshapes
            for (size_t j = 0; j < keyNames.size(); ++j) {
                const pcl::PointXYZ & keyPt_PCL = keyClouds[j]->points[point_index];
                initPt += vec_t(T(keyPt_PCL.x), T(keyPt_PCL.y), T(keyPt_PCL.z)) * _w[j];
            }

            // linear blend skinning (LBS)
            for (auto & bw : boneWeights[point_index]) {
                result += _toJointSpace(static_cast<JointType>(bw.first), initPt, _pt, _rt, _s) * T(bw.second);
            }
            return result;
        }

        Eigen::Vector3d computePointPosition(size_t point_index);

        // section MODEL PARAMETERS

        /** _w: shape key weights, _r : rotations, _s : scale, _p : global position */
        double * _w, *_r, *_s, _p[3];

        // internal parameter storage
        double * _pb, *_rb, *_pt, *_rt;
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
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> keyClouds;
        std::vector<std::string> keyNames;

        friend struct Joint;
    };
}
