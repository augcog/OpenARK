#include "stdafx.h"
#include "util/Visualizer.h"
#include "util/Util.h"
#include "hand_and_avatar/avatar/Avatar.h"
#include "hand_and_avatar/avatar/HumanDetector.h"

namespace {
    typedef ark::HumanAvatar::EigenCloud_T cloud;
    typedef ark::HumanAvatar::JointType smpl_j;
    typedef ark::HumanDetector::OpenPoseMPIJoint mpi_j;
}

namespace ark {
    /** UKF Model for HumanAvatar */
    struct HumanAvatarUKFModel 
    {
        /** define state vector */
        typedef kalman::Vector<0,         // scalars (none used)
            HumanAvatar::NUM_JOINTS + 2,  // 3-vectors (joint ang. vel. + root pos, root vel)
            HumanAvatar::NUM_JOINTS>      // quaternions (one for each joint)
            StateVec;

        /** define measurement vector */
        typedef kalman::Vector<0,            // measured scalars (none used)
            HumanAvatar::NUM_JOINTS,         // measured 3-vectors
            0>                               // measured quaternions (none used)
            MeasureVec;

        /** initializer */
        static void init(kalman::UKF<HumanAvatarUKFModel, HumanAvatar> & ukf) 
        {
            ukf.defaultInitialize(1e-5, 5e-3, 1e-3);
            // manually set the state root cov
            ukf.stateRootCov.diagonal().template segment<3>(StateVec::QUAT_START - 6).setConstant(1e-6);
            ukf.stateRootCov.diagonal().template segment<3>(StateVec::QUAT_START - 3).setConstant(5e-6);
            for (int i = 0; i < StateVec::_NUM_QUATERNIONS; ++i) {
                ukf.stateRootCov.diagonal().template segment<3>(StateVec::QUAT_START + i * 3) << 1e-5, 1e-5, 1e-5;
            }

            // manually set the process noise cov
            ukf.processNoiseRootCov.diagonal().template segment<3>(StateVec::QUAT_START - 6)
                                                .setConstant(1e-4);
            ukf.processNoiseRootCov.diagonal().template segment<3>(StateVec::QUAT_START - 3)
                                                .setConstant(1e-4);
        }

        /** process model definition: first derivative */
        static StateVec dF(const StateVec & state, const HumanAvatar & input) 
        {
            StateVec out = state;
            ark::kalman::util::diffQuaternion(out, 0);
            ark::kalman::util::diffPosition(out, StateVec::QUAT_START - 6,
                StateVec::QUAT_START - 3, StateVec::QUAT_START - 3);
            return out;
        }

        /* measurement model definition */
        static MeasureVec H(const StateVec & state, const HumanAvatar & input) 
        {

            static double pb[HumanAvatar::NUM_JOINTS * 3],
                rt[HumanAvatar::NUM_JOINTS * 4], cache[HumanAvatar::NUM_JOINTS * 9];

            const double * r = state.data() + StateVec::QUAT_START,
                *p = state.data() + StateVec::QUAT_START - 6,
                *w = input.w();

            MeasureVec m = MeasureVec::Zero();
            double * pt = m.data();
            input._propagateJointTransforms(r, p, w, pb, pt, rt, cache);
            return m;
        }
    };


    HumanAvatar::Joint::Joint(HumanAvatar & avatar, JointType type) :
        avatar(avatar), type(type),
        rotation(avatar._r + NUM_ROT_PARAMS * type),
        posBase(avatar._pb + NUM_POS_PARAMS * type),
        posTransformed(avatar._pt + NUM_POS_PARAMS * type),
        rotTransformed(avatar._rt + NUM_ROT_PARAMS * type),
        cachedTransform(avatar._cache + NUM_ROT_MAT_PARAMS * type)
    {
        rotation = Eigen::Quaterniond::Identity();
        cachedTransform = Eigen::Matrix3d::Identity();
    }

    const std::pair<int, int> HumanAvatar::MATCHED_JOINTS[] = 
    {
        //{ smpl_j::L_HIP, mpi_j::LEFT_HIP },
        //{ smpl_j::R_HIP, mpi_j::RIGHT_HIP },
        { smpl_j::L_KNEE, mpi_j::LEFT_KNEE },
        { smpl_j::R_KNEE, mpi_j::RIGHT_KNEE },
        { smpl_j::L_ANKLE, mpi_j::LEFT_ANKLE },
        { smpl_j::R_ANKLE, mpi_j::RIGHT_ANKLE },
        { smpl_j::NECK, mpi_j::NECK },
        { smpl_j::L_ELBOW, mpi_j::LEFT_ELBOW },
        { smpl_j::R_ELBOW, mpi_j::RIGHT_ELBOW },
        { smpl_j::L_WRIST, mpi_j::LEFT_WRIST },
        { smpl_j::R_WRIST, mpi_j::RIGHT_WRIST }
    };
    const int HumanAvatar::NUM_MATCHED_JOINTS = static_cast<int>(sizeof HumanAvatar::MATCHED_JOINTS /
                                                                   sizeof HumanAvatar::MATCHED_JOINTS[0]);


    HumanAvatar::HumanAvatar(const std::string & model_dir, int downsample_factor) : 
        HumanAvatar(model_dir, std::vector<std::string>(), downsample_factor) { }

    HumanAvatar::HumanAvatar(const std::string & model_dir, const std::vector<std::string> & shape_keys,
        double downsample_radius)
        : MODEL_DIR(model_dir), keyNames(shape_keys), basePos(_p) 
    {

        humanPCBase = std::unique_ptr<Cloud_T>(new Cloud_T());
        humanPCTransformed = boost::make_shared<Cloud_T>();

        using namespace boost::filesystem;
        path modelPath(model_dir); modelPath = modelPath / "model.pcd";
        path skelPath(model_dir); skelPath = skelPath / "skeleton.txt";
        path jrPath(model_dir); jrPath = jrPath / "joint_regressor.txt";
        path priorPath(model_dir); priorPath = priorPath / "pose_prior.txt";

        _w = new double[shape_keys.size()];
        memset(_w, 0, shape_keys.size() * sizeof(double));

        auto humanPCRaw = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto humanPCDown = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto humanPCFull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile<pcl::PointXYZ>(modelPath.string(), *humanPCFull);

        std::ifstream jr(jrPath.string());

        int nJoints, nVerts;
        std::vector<int> critVerts;
        jr >> nJoints;
        jointRegressor.resize(nJoints);
        for (int i = 0; i < nJoints; ++i) {
            int nEntries; jr >> nEntries;
            jointRegressor[i].resize(nEntries);
            for (int j = 0; j < nEntries; ++j) {
                jr >> jointRegressor[i][j].first >> jointRegressor[i][j].second;
                critVerts.push_back(jointRegressor[i][j].first);
            }
        }

        posePrior.load(priorPath.string());

        pcl::UniformSampling<pcl::PointXYZ> uniform_downsampler;
        uniform_downsampler.setInputCloud(humanPCFull);
        uniform_downsampler.setRadiusSearch(0.05);
        uniform_downsampler.filter(*humanPCDown);

        std::vector<int> introns;
        for (int i = 0; i < humanPCFull->points.size(); ++i) {
            for (int j = 0; j < humanPCDown->points.size(); ++j) {
                auto full_pt = humanPCFull->points[i];
                auto down_pt = humanPCDown->points[j];
                if (full_pt.x == down_pt.x && full_pt.y == down_pt.y && full_pt.z == down_pt.z) {
                    introns.push_back(i);
                }
            }
        }
        // add "critical" vertices used for SMPL joint regression
        std::copy(critVerts.begin(), critVerts.end(), std::back_inserter(introns));
        std::sort(introns.begin(), introns.end());

        // sort and remove duplicates
        introns.resize(std::unique(introns.begin(), introns.end()) - introns.begin());
        for (auto i : introns) {
            humanPCRaw->push_back(humanPCFull->points[i]);
        }

#ifdef DEBUG
        std::cerr << "HumanPCFull Size: " << humanPCFull->size() << endl;
        std::cerr << "HumanPCRaw Size: " << humanPCRaw->size() << endl;
#endif

        // coordinate compression
        for (int i = 0; i < nJoints; ++i) {
            for (size_t j = 0; j < jointRegressor[i].size(); ++j) {
                int & v = jointRegressor[i][j].first;
                v = std::lower_bound(introns.begin(), introns.end(), v) - introns.begin();
            }
        }

#ifdef DEBUG
        std::cerr << "Introns Size: " << introns.size() << endl;
#endif

        // load all required shape keys
        path keyPath(model_dir); keyPath = keyPath / "shapekey";
        for (std::string k : shape_keys) {
            auto keyPC = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
            pcl::io::loadPCDFile<pcl::PointXYZ>((keyPath / k).string(), *keyPC);
            // Ceiling division to compute the number of points in the down sampled cloud
            //EigenCloud_T keyCloud((keyPC->points.size() + downsample_factor - 1 )/ downsample_factor, 3);
            EigenCloud_T keyCloud(introns.size(), 3);
            for (size_t i = 0; i < introns.size(); ++i) {
                keyCloud.row(i) = keyPC->points[introns[i]].getVector3fMap().cast<double>();
            }
            keyClouds.push_back(keyCloud);
#ifdef DEBUG
            std::cerr << "Key Cloud: " << ii << endl;
#endif
        }

        // read skeleton file
        std::ifstream skel(skelPath.string());
        skel >> nJoints >> nVerts;

        // initialize parameter vectors
        _r = new double[nJoints * NUM_ROT_PARAMS];
        _rr = new double[nJoints * NUM_POS_PARAMS];
        _pb = new double[nJoints * NUM_POS_PARAMS];
        _pt = new double[nJoints * NUM_POS_PARAMS];
        _rt = new double[nJoints * NUM_ROT_PARAMS];
        _cache = new double[nJoints * NUM_ROT_MAT_PARAMS];

        // assume joints are given in topologically sorted order
        for (int i = 0; i < nJoints; ++i) {
            int id, parID; double x, y, z;
            skel >> id >> parID;

            auto j = std::make_shared<Joint>(*this, (JointType)id);
            skel >> j->name >> x >> y >> z;
            j->posSkel = Eigen::Vector3d(x, y, z);
            j->posTransformed = j->posBase = j->posSkel;
            if (parID != -1) {
                j->parent = joints[parID].get();
                j->parent->children.push_back(j.get());
            }
            else j->parent = nullptr;
            joints.push_back(j);
        }

        boneWeights.resize(introns.size());

        int ii = 0;
        // true if the model already provides vertex weights (else we calculate them ourselves)
        bool modelProvidesVertWeights = static_cast<bool>(skel);
        if (modelProvidesVertWeights) {
            for (int i = 0; i < nVerts; ++i) {

                int nEntries; skel >> nEntries;
                // skip appropriate number of points if downsample is enabled
                if (!std::binary_search(introns.begin(), introns.end(), i)) {
                    for (int j = 0; j < nEntries; ++j) {
                        int joint; double w; skel >> joint >> w;
                    }
                    continue;
                }
                
                // need to convert vertex (joint) weights to bone weights
                double total = 0.0;
                for (int j = 0; j < nEntries; ++j) {
                    int joint; double w;
                    skel >> joint >> w;
                    boneWeights[ii].push_back(std::make_pair(joint, w));
                    total += w;
                }

                // normalize weights to add to 1
                for (auto & p : boneWeights[ii]) {
                    p.second /= total;
                }
                ii++;
            }
        }

        // use XYZRGBA point format
        pcl::copyPointCloud(*humanPCRaw, *humanPCTransformed);

        // store initial configuration so we can easily return to it
        pcl::copyPointCloud(*humanPCTransformed, *humanPCBase);

        // propagate initial joint transforms (since meta file provides local position)
        propagateJointTransforms();

        if (!modelProvidesVertWeights) {
            std::cerr << "WARNING: no vertex weights found in avatar model." <<
                "Initializing based on distance...\n";
            // assign skeleton weights using distance metric
            assignDistanceWeights();
        }

        // color the point cloud
        colorByWeights();
    }

    HumanAvatar::~HumanAvatar() 
    {
        // clean up parameter vectors
        delete[] _w; delete[] _r; delete[] _rr;
        delete[] _pb; delete[] _pt; delete[] _rt;
        delete[] _cache;
    }

    // section GETTERS/SETTERS

    // model parameter vector getters/setters
    /** Get the avatar's key weight parameter vector
    * (ith index is relative weight of ith shape key a.k.a. blendshape;
    *  total size is numKeys()) */
    double * HumanAvatar::w() { return _w; }
    const double * HumanAvatar::w() const { return _w; }

    /** Get the avatar's bone rotation parameter vector (stores quaternions: x y z w)
    * (first 4 indices are root rotation quaternion, total size is numJoints() * 4) */
    double * HumanAvatar::r() { return _r; }
    const double * HumanAvatar::r() const { return _r; }

    /** Get the avatar's global position (length 3) */
    double * HumanAvatar::p() { return _p; }
    const double * HumanAvatar::p() const { return _p; }

    Eigen::VectorXd HumanAvatar::smplParams() const
    {
        return _smplParams(_r);
    }

    Eigen::Map<Eigen::Vector3d> HumanAvatar::getBasePosition() 
    {
        return basePos;
    }

    Eigen::Vector3d HumanAvatar::getUndeformedBoneVector(int joint_id) 
    {
        if (joints[joint_id]->parent == nullptr) return Eigen::Vector3d(0, 0, 0);
        return joints[joint_id]->posBase - joints[joint_id]->parent->posBase;
    }

    Eigen::Vector3d HumanAvatar::getBoneVector(int joint_id) 
    {
        if (joints[joint_id]->parent == nullptr) return Eigen::Vector3d(0, 0, 0);
        return joints[joint_id]->posTransformed - joints[joint_id]->parent->posTransformed;
    }

    const Eigen::Map<Eigen::Vector3d> & HumanAvatar::getPosition(int joint_id) const 
    {
        return joints[joint_id]->posTransformed;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getLocalRotation(int joint_id)
    {
        return joints[joint_id]->rotation;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getCenterRotation() 
    {
        return joints[JointType::ROOT_PELVIS]->rotation;
    }

    void HumanAvatar::setCenterPosition(const Eigen::Vector3d & val) 
    {
        basePos = val;
    }

    void HumanAvatar::setRotation(int joint_id, const Eigen::Quaterniond & quat) 
    {
        joints[joint_id]->rotation = quat.normalized();
    }

    /** Set the local rotation of the bone ending at a joint to the given AngleAxis object */
    void HumanAvatar::setRotation(int joint_id, const Eigen::AngleAxisd & angle_axis) 
    {
        joints[joint_id]->rotation = Eigen::Quaterniond(angle_axis);
    }

    /** Set the local rotation of the bone ending at a joint to the given euler angles */
    void HumanAvatar::setRotation(int joint_id, double yaw, double pitch, double roll) 
    {
        joints[joint_id]->rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        joints[joint_id]->rotation.normalize();
    }

    /** Set the local rotation of the bone ending at a joint so that v1 in the original space rotates to v2 */
    void HumanAvatar::setRotation(int joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2) 
    {
        Eigen::Map<Eigen::Quaterniond> & q = joints[joint_id]->rotation;
        v1.normalize(); v2.normalize();
        double dot = v1.dot(v2);
        if (dot > 0.9999 || dot < -0.9999) {
            q = Eigen::Quaterniond::Identity();
        }
        else {
            q = Eigen::AngleAxisd(acosf(dot), v1.cross(v2));
            q.normalize();
        }
    }

    /** Adds a rotation to the local rotation of the bone ending at a joint */
    void HumanAvatar::_addRotation(int joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2) 
    {
        v1.normalize(); v2.normalize();
        double dot = v1.dot(v2);
        if (dot <= 0.9999 && dot >= -0.9999) {
            Eigen::Quaterniond q;
            q.vec() = v1.cross(v2);
            q.w() = 1.0 + dot;
            q.normalize();
            joints[joint_id]->rotation = q * joints[joint_id]->rotation;
            joints[joint_id]->rotation.normalize();
        }
    }

    /** Set the local rotation of the bone ending at a joint so that it points to v */
    void HumanAvatar::setRotation(int joint_id, const Eigen::Vector3d & v) 
    {
        Eigen::Vector3d v1 = getBoneVector(joint_id);
        _addRotation(joint_id, v1, v);
    }

    /** Get a pointer to the specified joint */
    HumanAvatar::Joint::Ptr HumanAvatar::getJoint(int joint_id) const 
    {
        return joints[joint_id];
    }

    Eigen::Vector3d HumanAvatar::getJointPosition(int joint_id) const 
    {
        return joints[joint_id]->posTransformed;
    }

    Eigen::Vector2d HumanAvatar::getJointPosition2d(int joint_id) const 
    {
        return HumanDetector::projectToImage(pinholeIntrin, joints[joint_id]->posTransformed);
    }

    /** Get the number of joints in the avatar's skeleton */
    int HumanAvatar::numJoints() const 
    {
        return (int)joints.size();
    }

    /** Get the weight of the given shape key (blendshape) */
    double & HumanAvatar::getKeyWeight(int id) 
    {
        return _w[id];
    }

    /** Get the name of the given shape key (blendshape) */
    const std::string & HumanAvatar::getKeyName(int id) const 
    {
        return keyNames[id];
    }

    /** Set the weight of the given shape key (blendshape) */
    void HumanAvatar::setKeyWeight(int id, double weight) 
    {
        _w[id] = weight;
    }

    /** Get the number of shape keys (blendshapes) available */
    int HumanAvatar::numKeys() const 
    {
        return (int)keyNames.size();
    }

    void HumanAvatar::reset(bool update) 
    {
        memset(_p, 0, NUM_POS_PARAMS * sizeof(_p[0]));
        memset(_w, 0, NUM_SHAPEKEYS * sizeof(_w[0]));
        for (int i = 0; i < joints.size(); ++i) {
            joints[i]->rotation = Eigen::Quaterniond::Identity();
        }
        if (update) {
            this->update();
        }
    }

    HumanAvatar::Cloud_T::Ptr HumanAvatar::getCloud(bool update) 
    {
        if (update) this->update();
        return humanPCTransformed;
    }

    void HumanAvatar::update(bool propagate) 
    {
        if (propagate) propagateJointTransforms();

        for (size_t i = 0; i < humanPCTransformed->points.size(); ++i) 
        {
            humanPCTransformed->points[i].getVector3fMap() = computePointPosition(i).cast<float>();
        }
    }

    void HumanAvatar::colorByWeights() 
    {
        for (int i = 0; i < (int)humanPCTransformed->points.size(); ++i) 
        {
            humanPCTransformed->points[i].rgb = 0;
            // color based on weights
            for (int j = 0; j < (int)boneWeights[i].size(); ++j) 
            {
                Vec3b color = util::paletteColor(boneWeights[i][j].first, false);
                humanPCTransformed->points[i].r += color[0] * boneWeights[i][j].second;
                humanPCTransformed->points[i].g += color[1] * boneWeights[i][j].second;
                humanPCTransformed->points[i].b += color[2] * boneWeights[i][j].second;
            }
        }
    }

    void HumanAvatar::color(std::vector<std::vector<int>> & groups) 
    {
        for (int g = 0; g < groups.size(); ++g) 
        {
            Vec3b color = util::paletteColor(g, false);
            for (int i : groups[g]) 
            {
                humanPCTransformed->points[i].r = color[0];
                humanPCTransformed->points[i].g = color[1];
                humanPCTransformed->points[i].b = color[2];
            }
        }
    }

    // helper functions, etc. for ceres solver
    /** Update the specified eigen point cloud to reflect changes to the joints
    * @param propagate if true, automatically propagates changes throughout the skeleton.
    *       (default is true)
    */
    template<class T, int opt>
    void HumanAvatar::_updateCloud(const T * const _w, T * _pt, T * _cache, Eigen::Matrix<T, -1, 3, opt> & out) {
        for (size_t i = 0; i < out.rows(); ++i) {
            out.row(i) = _computePointPosition(i, _w, _pt, _cache).template cast<T>();
        }
    }

    /** Compute the avatar's SMPL pose parameters (Rodrigues angles) */
    template<class T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> HumanAvatar::_smplParams(const T * const _r) const
    {
        Eigen::Matrix<T, Eigen::Dynamic, 1> res;
        res.resize((NUM_JOINTS - 1) * 3);
        for (int i = 1; i < NUM_JOINTS; ++i) 
        {
            const Eigen::Map<const Eigen::Quaternion<T>> q(_r + i * NUM_ROT_PARAMS);

            T n = q.vec().norm();
            if (n == T(0)) 
            {
                res.template segment<3>((i - 1) * 3).setZero();
            }
            else 
            {
                if (q.w() < T(0)) n = -n;
                res.template segment<3>((i - 1) * 3) = q.vec() / n * T(2) * ceres::atan2(n, abs(q.w()));
            }
        }
        return res;
    }

    HumanAvatar::kd_tree_ptr_t HumanAvatar::_buildKDIndex(const EigenCloud_T & dataCloud)
    {
        using namespace nanoflann;
        auto out = std::make_shared<kd_tree_t>(3, dataCloud, 10);
        out->index->buildIndex();
        return out;
    }

    void HumanAvatar::_findNN(const kd_tree_ptr_t & mindex, const EigenCloud_T & dataCloud, const EigenCloud_T & modelCloud,
        std::vector<std::pair<int, int>> & correspondences, bool inverted) 
    {

        size_t index; double dist;
        nanoflann::KNNResultSet<double> resultSet(1);
        
        if (inverted) 
        {
            // perform INVERSE NN, i.e. match each data point to a model point
            kd_tree_t mindex(3, modelCloud, 10);
            mindex.index->buildIndex();

            std::vector<std::vector<int>> neighb(modelCloud.rows());

            correspondences.clear();
            for (int i = 0; i < dataCloud.rows(); ++i) 
            {
                resultSet.init(&index, &dist);
                mindex.index->findNeighbors(resultSet, dataCloud.data() + i * 3, nanoflann::SearchParams(10));
                neighb[int(index)].emplace_back(i);
            }

            // limit to 1 NN point per model point
            for (int i = 0; i < modelCloud.rows(); ++i) 
            {
                int best_idx = -1; double best_norm = DBL_MAX;
                for (int nei : neighb[i]) 
                {
                    double norm = (dataCloud.row(nei) - modelCloud.row(i)).squaredNorm();
                    if (norm < best_norm) 
                    {
                        best_norm = norm;
                        best_idx = nei;
                    }
                }
                if (~best_idx) 
                {
                    correspondences.emplace_back(i, best_idx);
                }
            }
        } 
        else 
        {
            // perform FORWARD NN, i.e. match each model point to a data point
            correspondences.clear();
            std::vector<std::vector<int>> invNN(dataCloud.rows());

            const double * dataPtr = modelCloud.data();
            for (int i = 0; i < modelCloud.rows(); ++i) 
            {
                resultSet.init(&index, &dist);
                mindex->index->findNeighbors(resultSet, dataPtr, nanoflann::SearchParams(10));
                dataPtr += 3;
                if (index >= 0 && index < static_cast<int>(invNN.size()))
                    invNN[index].push_back(i);
            }

            // limit to 1 NN point per data point
            for (int i = 0; i < dataCloud.rows(); ++i) 
            {
                if (invNN[i].size() > 0) 
                {
                    int best_idx = 0; double best_norm = DBL_MAX;
                    for (int j = 0; j < invNN[i].size(); ++j) 
                    {
                        double norm = (dataCloud.row(i) - modelCloud.row(invNN[i][j])).squaredNorm();
                        if (norm < best_norm) 
                        {
                            best_norm = norm;
                            best_idx = invNN[i][j];
                        }
                    }
                    correspondences.emplace_back(best_idx, i);
                }
            }
        }
    }

    // function called at each iteration of optimization procedure, used for debugging
    static void __debugVisualize(HumanAvatar * ava, const HumanAvatar::EigenCloud_T & dataCloud,
        const HumanAvatar::EigenCloud_T & modelCloud, const std::vector<std::pair<int, int>> & correspondences,
                                                      bool print_params = false) 
    {

        const int NUM_JOINTS = ava->numJoints();

        if (print_params) 
        {
            // print out model parameters
            double * _w = ava->w(), *_r = ava->r(), *_p = ava->p();
            for (int i = 0; i < HumanAvatar::NUM_SHAPEKEYS; ++i) 
            {
                cout << _w[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < NUM_JOINTS * HumanAvatar::NUM_ROT_PARAMS; ++i) 
            {
                cout << _r[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < HumanAvatar::NUM_POS_PARAMS; ++i) 
            {
                cout << _p[i] << " ";
            }
            cout << "\n";
        }

        const auto & viewer = Visualizer::getPCLVisualizer();
        viewer->removeAllShapes(1);

        // draw nearest-neighbor lines
        int lineInterval = std::max(2, int(correspondences.size()) / 100);
        for (size_t k = 0; k < correspondences.size(); k += lineInterval) {
            int i, j; std::tie(i, j) = correspondences[k];
            HumanAvatar::Point_T p1, p2;
            p1.getVector3fMap() = modelCloud.row(i).cast<float>();
            p2.getVector3fMap() = dataCloud.row(j).cast<float>();
            std::string name = "nn_line_" + std::to_string(j);
            viewer->addLine<HumanAvatar::Point_T, HumanAvatar::Point_T>(p2, p1, 1.0, 0.0, 0.0, name, 1);
        }

        // re-draw model joints and points
        ava->update(false);
        ava->visualize(viewer, "ava_", 1);
        //viewer->spinOnce();
    }

    void HumanAvatar::fit(const EigenCloud_T & dataCloud, double deltat, bool track) 
    {
        static kalman::UKF<HumanAvatarUKFModel, HumanAvatar> ukf;

        auto startTime = std::chrono::high_resolution_clock::now();
        kd_tree_ptr_t kdTree = _buildKDIndex(dataCloud);
        std::vector<int> joints_subset;
        if (!track) 
        {
            fitPose(dataCloud, 1, 2, joints_subset, true, kdTree);
            fitShape(dataCloud, 1, 14, true, kdTree);
            fitPose(dataCloud, 10, 12, joints_subset, true, kdTree);
            fitShape(dataCloud, 3, 14, true, kdTree);
            fitPose(dataCloud, 3, 5, joints_subset, true, kdTree);

            // manually reset state to estimate
            ukf.state.template head<NUM_JOINTS * 3>().setZero(); // ang vels
            ukf.state.template segment<3>(NUM_JOINTS * 3) = basePos; // pos
            ukf.state.template segment<3>(NUM_JOINTS * 3 + 3).setZero(); // vel
            Eigen::Map<Eigen::Matrix<double, NUM_JOINTS * 4, 1> > rots(_r); 
            ukf.state.template segment<NUM_JOINTS * 4>(NUM_JOINTS * 3 + 6) = rots; // rotations
        }
        else 
        {
            fitPose(dataCloud, 2, 4, joints_subset, true, kdTree);

            // update ukf
            Eigen::Map<Eigen::Matrix<double, NUM_JOINTS*3, 1>> z(_pt);
            if (deltat < 0) 
            {
                deltat = 1./40.;// default interval between frames in our dataset
            }
            ukf.update(deltat, static_cast<HumanAvatarUKFModel::MeasureVec>(z), *this);
        }

        // use ukf state to overwrite avatar state
        /*
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS * 4, 1>> rots(_r);
        rots = ukf.state.template tail<NUM_JOINTS * 4>();
        basePos = ukf.state.template segment<3>(NUM_JOINTS * 3);
        update(true);
*/

        auto endTime = std::chrono::high_resolution_clock::now();
        std::cout << "Overall Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << " ms\n";

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        std::vector<std::pair<int, int> > correspondences;
        _updateCloud(_w, _pt, _cache, modelCloud);
        _findNN(kdTree, dataCloud, modelCloud, correspondences, false);
        __debugVisualize(this, dataCloud, modelCloud, correspondences, false);
    }

    void HumanAvatar::fitPose(const EigenCloud_T & dataCloud, int max_iter, int num_subiter,
        const std::vector<int> & joint_subset, bool inv_nn, kd_tree_ptr_t kd_tree) 
    {
        using namespace ceres;

        kd_tree_ptr_t kdTree = (kd_tree ? kd_tree : _buildKDIndex(dataCloud));

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);

        EigenCloud_T initJointPos(NUM_JOINTS, 3);
        for (int i = 0; i < NUM_JOINTS; ++i) 
        {
            initJointPos.row(i) = Eigen::Map<Eigen::Vector3d>(_pt + i * NUM_POS_PARAMS);
        }

        for (int iter = 0; iter < max_iter; ++iter) 
        {
            std::cout << ">> POSE FITTING: ITER " << iter << "\n";
            _propagateJointTransforms(_r, _p, _w, _pb, _pt, _rt, _cache);
            _updateCloud(_w, _pt, _cache, modelCloud);

            Problem problem;
            std::vector<std::pair<int, int> > correspondences;

            // find nearest neighbors using nanoflann kd tree
            _findNN(kdTree, dataCloud, modelCloud, correspondences, inv_nn);
            ceres::CostFunction * cost_function =
                new AutoDiffCostFunction<PoseCostFunctor, ceres::DYNAMIC,
                NUM_JOINTS * NUM_ROT_PARAMS,
                NUM_POS_PARAMS>(
                    new PoseCostFunctor(*this, dataCloud, correspondences, jointsPrior,
                                        pinholeIntrin, posePrior),
                            int(correspondences.size()) * NUM_POS_PARAMS
                        + NUM_MATCHED_JOINTS * 3
                        + NUM_JOINTS * 3 - 2
                    );

            problem.AddParameterBlock(_r, NUM_JOINTS * NUM_ROT_PARAMS
                , new MultiQuaternionParameterization<NUM_JOINTS>()
            );
            problem.AddParameterBlock(_p, NUM_POS_PARAMS);
            problem.AddResidualBlock(cost_function, NULL /*new CauchyLoss(25.0)*/, _r, _p);

            Solver::Options options;
            options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            options.initial_trust_region_radius = 1e2;
            options.minimizer_progress_to_stdout = false;
            options.logging_type = ceres::LoggingType::SILENT;
            options.minimizer_type = ceres::TRUST_REGION;
            options.preconditioner_type = ceres::PreconditionerType::JACOBI;
            //options.line_search_direction_type = ceres::LBFGS;
            options.max_linear_solver_iterations = num_subiter;
            options.max_num_iterations = num_subiter;
            options.num_threads = 1;
            options.function_tolerance = 1e-5;

            // solve iteratively
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)

            //__debugVisualize(this, dataCloud, modelCloud, correspondences);
            //std::cout << summary.FullReport() << "\n";
        }

        _propagateJointTransforms(_r, _p, _w, _pb, _pt, _rt, _cache);
        _updateCloud(_w, _pt, _cache, modelCloud);
        std::cout << ">> POSE FITTING: DONE\n";
    }

    void HumanAvatar::fitShape(const EigenCloud_T & dataCloud, int max_iter, int num_subiter, bool inv_nn, kd_tree_ptr_t kd_tree) 
    {
        using namespace ceres;

        kd_tree_ptr_t kdTree = (kd_tree ? kd_tree : _buildKDIndex(dataCloud));

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);

        for (int iter = 0; iter < max_iter; ++iter) 
        {
            std::cout << ">> SHAPE FITTING: ITER " << iter << "\n";
            _propagateJointTransforms(_r, _p, _w, _pb, _pt, _rt, _cache);
            _updateCloud(_w, _pt, _cache, modelCloud);

            Problem problem;
            std::vector<std::pair<int, int> > correspondences;
            // find nearest neighbors using nanoflann kd tree
            _findNN(kdTree, dataCloud, modelCloud, correspondences, inv_nn);
            ceres::CostFunction * cost_function =
                new AutoDiffCostFunction<ShapeCostFunctor, ceres::DYNAMIC,
                NUM_SHAPEKEYS
                //NUM_JOINTS * NUM_ROT_PARAMS,
                //NUM_JOINTS * NUM_SCALE_PARAMS,
                /*NUM_POS_PARAMS*/>(
                    new ShapeCostFunctor(*this, dataCloud, correspondences, jointsPrior, pinholeIntrin),
                    int(correspondences.size()) * NUM_POS_PARAMS
                    + NUM_MATCHED_JOINTS * 3
                    + NUM_SHAPEKEYS);

            problem.AddParameterBlock(_w, NUM_SHAPEKEYS);
            problem.AddResidualBlock(cost_function, NULL /*new CauchyLoss(25.0)*/, _w);

            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = false;
            options.logging_type = ceres::SILENT;
            options.initial_trust_region_radius = 200;
            options.minimizer_type = ceres::TRUST_REGION;
            //options.line_search_direction_type = ceres::LBFGS;
            options.max_linear_solver_iterations = num_subiter;
            options.max_num_iterations = num_subiter;
            options.num_threads = 1;
            //options.num_linear_solver_threads = 1;
            options.function_tolerance = 1e-8;

            // __debugVisualize(this, dataCloud, modelCloud, correspondences);

            // solve ICP
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)
            // std::cout << summary.FullReport() << "\n";

            // Visualizer::getPCLVisualizer()->spinOnce(500);
        }

        _propagateJointTransforms(_r, _p, _w, _pb, _pt, _rt, _cache);
        std::cout << ">> SHAPE FITTING: DONE\n";
    }

    void HumanAvatar::alignToJoints(const EigenCloud_T & pos)
    {
        ARK_ASSERT(pos.rows() == JointType::_COUNT, "Joint number mismatch");

        Eigen::Vector3d vr = joints[JointType::SPINE1]->posSkel - joints[JointType::ROOT_PELVIS]->posSkel;
        Eigen::Vector3d vrt = pos.row(JointType::SPINE1) - pos.row(JointType::ROOT_PELVIS);
        basePos = pos.row(0);
        joints[JointType::ROOT_PELVIS]->rotTransformed = joints[JointType::ROOT_PELVIS]->rotation
                                                = Eigen::Quaterniond::FromTwoVectors(vr, vrt);

        double scaleAvg = 0.0;
        for (int i = 1; i < pos.rows(); ++i) 
        {
            scaleAvg += (pos.row(i) - pos.row(joints[i]->parent->type)).norm() /
                (joints[i]->posSkel - joints[i]->parent->posSkel).norm();
        }
        scaleAvg /= (pos.rows() - 1.0);
        double baseScale = (joints[JointType::SPINE2]->posSkel - joints[JointType::ROOT_PELVIS]->posSkel).norm() * (scaleAvg - 1.0);
        _w[0] = baseScale * PC1_DIST_FACT;
        if (isnan(_w[0])) _w[0] = 1.5;
        propagateJointTransforms();

        for (int i = 1; i < pos.rows(); ++i) 
        {
            joints[i]->rotation = Eigen::Quaterniond::Identity();
            joints[i]->rotTransformed = joints[i]->parent->rotTransformed;
            joints[i]->cachedTransform = joints[i]->rotTransformed.toRotationMatrix();
            if (joints[i]->children.empty() || std::isnan(pos.row(i).x()) || std::isnan(pos.row(joints[i]->children[0]->type).x())) 
            {
            }
            else 
            {
                Eigen::Vector3d vv = joints[i]->children[0]->posSkel - joints[i]->posSkel;
                Eigen::Vector3d vvt = pos.row(joints[i]->children[0]->type) - pos.row(i);
                joints[i]->rotation = joints[i]->parent->rotTransformed.inverse() * Eigen::Quaterniond::FromTwoVectors(vv, vvt);
                joints[i]->rotTransformed = joints[i]->parent->rotTransformed * joints[i]->rotation;
                joints[i]->cachedTransform = joints[i]->rotTransformed.toRotationMatrix();
            }
        }
    }

    void HumanAvatar::updateJointsPrior(const EigenCloud_T & pos)
    {
        jointsPrior = pos;
    }

    void HumanAvatar::updateCameraIntrin(const cv::Vec4d & intrin)
    {
        pinholeIntrin = intrin;
    }

    void HumanAvatar::visualize(const pcl::visualization::PCLVisualizer::Ptr & viewer, std::string pcl_prefix, int viewport) const 
    {
        for (int i = 0; i < joints.size(); ++i) 
        {
            Point_T curr = util::toPCLPoint(joints[i]->posTransformed);
            //std::cerr << "Joint:" << joints[i]->name << ":" << curr.x << "," << curr.y << "," << curr.z << "\n";

            Vec3b color = util::paletteColor(i);
            Vec3f colorf = color / 255.0;
            std::string jointName = pcl_prefix + "avatarJoint" + std::to_string(i);
            viewer->removeShape(jointName, viewport);
            viewer->addSphere(curr, 0.02, colorf[2], colorf[1], colorf[0], jointName, viewport);

            if (joints[i]->parent) 
            {
                Point_T parent = util::toPCLPoint(joints[i]->parent->posTransformed);
                std::string boneName = pcl_prefix + "avatarBone" + std::to_string(i);
                viewer->removeShape(boneName, viewport);
                viewer->addLine(curr, parent, colorf[2], colorf[1], colorf[0], boneName, viewport);
            }
        }

        static const std::string MODEL_CLOUD_NAME = "model_cloud";
        viewer->removePointCloud(pcl_prefix + MODEL_CLOUD_NAME, viewport);
        viewer->addPointCloud<Point_T>(humanPCTransformed, pcl_prefix + MODEL_CLOUD_NAME, viewport);
    }

    void HumanAvatar::assignDistanceWeights(int max_vertex_bones, double norm_thresh) 
    {
        const size_t SZ = humanPCTransformed->points.size();
        boneWeights.resize(SZ);
        std::vector<std::pair<double, int>> tmp(joints.size() - 1);
        for (size_t i = 0; i < SZ; ++i) 
        {
            const auto & pt = humanPCTransformed->points[i];
            Eigen::Vector3d v(pt.x, pt.y, pt.z);
            for (size_t j = 1; j < joints.size(); ++j) 
            {
                if (joints[j]->parent == nullptr) continue;
                const auto & pb = joints[j]->posBase;
                double norm = (v - pb).norm();
                tmp[j - 1].first = norm;
                tmp[j - 1].second = (int)j;
            }
            std::sort(tmp.begin(), tmp.end());
            double totalWt = 0.0;
            for (int j = 0; j < max_vertex_bones; ++j) 
            {
                if (j && tmp[j].first > norm_thresh) break;
                boneWeights[i].push_back(std::make_pair(tmp[j].second,
                    1.0 / (tmp[j].first * tmp[j].first)
                ));
                totalWt += boneWeights[i][j].second;
            }

            for (size_t j = 0; j < boneWeights[i].size(); ++j) 
            {
                boneWeights[i][j].second /= totalWt;
            }
        }
    }

    Eigen::Vector3d HumanAvatar::toJointSpace(int joint_id, const Eigen::Vector3d & vec) 
    {
        return _toJointSpace(joint_id, vec, _pt, _cache);
    }

    void HumanAvatar::propagateJointTransforms() 
    {
        _propagateJointTransforms(_r, _p, _w, _pb, _pt, _rt, _cache);
    }

    Eigen::Vector3d HumanAvatar::computePointPosition(size_t point_index) 
    {
        return _computePointPosition(point_index, _w, _pt, _cache);
    }

    void GaussianMixture::load(const std::string & path)
    {
        ifstream ifs(path);
        ifs >> nComps >> nDims;

        // compute constants
        double sqrt_2_pi_n = ceres::pow(2 * M_PI, nDims * 0.5 );
        double log_sqrt_2_pi_n = nDims * 0.5 * std::log(2 * M_PI);
        weight.resize(nComps);
        consts.resize(nComps);
        consts_log.resize(nComps);
        for (int i = 0; i < nComps; ++i) 
        {
            // load weights
            ifs >> weight[i];
            consts_log[i] = log(weight[i]) - log_sqrt_2_pi_n;
            consts[i] = weight[i] / sqrt_2_pi_n;
        }

        mean.resize(nComps, nDims);
        for (int i = 0; i < nComps; ++i) 
        {
            for (int j = 0; j < nDims; ++j) 
            {
                // load mean vectors
                ifs >> mean(i, j);
            }
        }

        /** Cholesky decomposition */
        typedef Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> Cholesky;

        cov.resize(nComps);
        cov_cho.resize(nComps);
        double maxDet = 0.0;
        for (int i = 0; i < nComps; ++i) 
        {
            auto & m = cov[i];
            m.resize(nDims, nDims);
            for (int j = 0; j < nDims; ++j) 
            {
                for (int k = 0; k < nDims; ++k) 
                {
                    // load covariance matrices
                    ifs >> m(j, k);
                }
            }
            Cholesky chol(cov[i].inverse());
            if (chol.info() != Eigen::Success) throw "Decomposition failed!";
            cov_cho[i] = chol.matrixL();
            double det = chol.matrixL().determinant();
            maxDet = std::max(det, maxDet);

            // update constants
            consts[i] *= det;
            consts_log[i] += log(det);
        }

        for (int i = 0; i < nComps; ++i) 
        {
            // normalize constants
            consts[i] /= maxDet;
            consts_log[i] -= log(maxDet);
        }
    }

    inline int GaussianMixture::numComponents() const
    { 
        return nComps; 
    }

    template<class T>
    /** Compute PDF at 'input' */
    T GaussianMixture::pdf(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x) const 
    {
        T prob(0.0);
        typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_t;
        for (int i = 0; i < nComps; ++i) 
        {
            Eigen::TriangularView<Eigen::MatrixXd, Eigen::Lower> L(cov_cho[i]);
            auto residual = (L.transpose() * (x - mean.row(i).transpose()));
            prob += consts[i] * ceres::exp(-0.5 * residual.squaredNorm());
        }
        return prob;
    }

    template<class T>
    /** Compute Ceres residual vector (squaredNorm of output vector is equal to min_i -log(c_i pdf_i(x))) */
    Eigen::Matrix<T, Eigen::Dynamic, 1> GaussianMixture::residual(const Eigen::Matrix<T, Eigen::Dynamic, 1> & x)
    {
        T bestProb = T(0);
        typedef Eigen::Matrix<T, Eigen::Dynamic, 1> vec_t;
        typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_t;
        vec_t ans;
        for (int i = 0; i < nComps; ++i) 
        {
            Eigen::TriangularView<Eigen::MatrixXd, Eigen::Lower> L(cov_cho[i]);
            vec_t residual(nDims + 1);
            residual[nDims] = T(0);
            residual.head(nDims) = L.transpose() * (x - mean.row(i).transpose()) * sqrt(0.5);
            T p = residual.squaredNorm() - T(consts_log[i]);
            if (p < bestProb || !i) 
            {
                bestProb = p;
                residual[nDims] = T(sqrt(-consts_log[i]));
                ans = residual;
            }
        }
        return ans;
    }

    template <typename T>
    bool HumanAvatar::PoseCostFunctor::operator()(const T* const r, const T* const p, T* residual) const {
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

        typedef Eigen::Matrix<T, 1, 3, Eigen::RowMajor> Vec3T;
        typedef Eigen::Map<Vec3T> Vec3TMap;
        typedef Eigen::Matrix<T, 1, 2, Eigen::RowMajor> Vec2T;
        typedef Eigen::Map<Vec2T> Vec2TMap;

        //T r1 = T(0), r2 = T(0), r3 = T(0);
        T * residualPtr = residual;
        for (auto & cor : correspondences) {
            // compute position of each point using transformed data
            Vec3T surfPoint = ava._computePointPosition(cor.first, w, pt, cache);
            Vec3TMap res(residualPtr);
            res = surfPoint - dataCloud.row(cor.second);
            //r1 += ceres::pow(res.x(),2);
            //r1 += ceres::pow(res.y(),2);
            //r1 += ceres::pow(res.z(),2);
            residualPtr += NUM_POS_PARAMS;
        }

        //Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor> joints2d;
        //static const int MPI_NJOINTS = HumanDetector::OpenPoseMPIJoint::_COUNT - 1;

        typedef Eigen::Map<Eigen::Matrix<T, NUM_JOINTS, 3, Eigen::RowMajor> > cloudMap;
        cloudMap currJointPositions(pt);
        //HumanDetector::toMPIJoints<T, cloudMap>(intrin, currJointPositions, joints2d);

        for (int i = 0; i < NUM_MATCHED_JOINTS; ++i) {
            Vec3TMap res(residualPtr);
            if (std::isnan(jointsPrior.row(i).x())) {
                res = Vec3T::Zero();
            }
            else {
                res = (currJointPositions.row(MATCHED_JOINTS[i].first)
                    - jointsPrior.row(i)) * (T)betaJ;
            }
            residualPtr += 3;
        }

        Eigen::Map<Eigen::Matrix<T, NUM_JOINTS * 3 - 2, 1>> posePriorResidual(residualPtr);
        posePriorResidual = posePrior.residual(ava._smplParams(r)) * (T)betaP;
        //for (int k = 0; k < NUM_JOINTS * 3 - 2; ++k) {
        //    r3 += ceres::pow(posePriorResidual[k], 2);
        //}
        //std::cerr << "[" << r1 << "\n " << r2 << "\n " << r3 << "]\n\n";
        return true;
    }

    template <typename T>
    bool HumanAvatar::ShapeCostFunctor::operator()(const T* const w, T* residual) const {
        const int NUM_JOINTS = HumanAvatar::JointType::_COUNT;
        // to fix parameters at initial value 
        T r[NUM_JOINTS * NUM_ROT_PARAMS], p[NUM_POS_PARAMS];
        for (int i = 0; i < NUM_JOINTS * NUM_ROT_PARAMS; ++i) r[i] = T(ava.r()[i]);
        for (int i = 0; i < NUM_POS_PARAMS; ++i) p[i] = T(ava.p()[i]);

        T pb[NUM_JOINTS * NUM_POS_PARAMS], pt[NUM_JOINTS * NUM_POS_PARAMS], rt[NUM_JOINTS * NUM_ROT_PARAMS], cache[NUM_JOINTS * NUM_ROT_MAT_PARAMS];
        // simulate propagation of skeletal transforms
        ava._propagateJointTransforms(r, p, w, pb, pt, rt, cache);

        typedef Eigen::Matrix<T, 1, 3, Eigen::RowMajor> Vec3T;
        typedef Eigen::Map<Vec3T> Vec3TMap;
        typedef Eigen::Matrix<T, 1, 2, Eigen::RowMajor> Vec2T;
        typedef Eigen::Map<Vec2T> Vec2TMap;

        T * residualPtr = residual;
        for (auto & cor : correspondences) {
            // compute position of each point using transformed data
            Vec3T surfPoint = ava._computePointPosition(cor.first, w, pt, cache);
            Vec3TMap res(residualPtr);
            res = surfPoint - dataCloud.row(cor.second);
            residualPtr += NUM_POS_PARAMS;
        }


        //Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor> joints2d;
        //static const int MPI_NJOINTS = HumanDetector::OpenPoseMPIJoint::_COUNT - 1;

        typedef Eigen::Map<Eigen::Matrix<T, NUM_JOINTS, 3, Eigen::RowMajor> > cloudMap;
        cloudMap currJointPositions(pt);
        //HumanDetector::toMPIJoints<T, cloudMap>(intrin, currJointPositions, joints2d);

        for (int i = 0; i < NUM_MATCHED_JOINTS; ++i) {
            Vec3TMap res(residualPtr);
            if (std::isnan(jointsPrior.row(i).x())) {
                res = Vec3T::Zero();
            }
            else {
                res = (currJointPositions.row(MATCHED_JOINTS[i].first)
                    - jointsPrior.row(i)) * (T)betaJ;
            }
            residualPtr += 3;
        }


        for (int i = 0; i < NUM_SHAPEKEYS; ++i) {
            residualPtr[i] = w[i] * betaShape;
        }

        return true;
    }


    template<int N>
    bool HumanAvatar::MultiQuaternionParameterization<N>::Plus(const double* x_ptr, const double* delta, double* x_plus_delta_ptr) const{
        for (int i = 0; i < N; ++i) {
            Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_ptr + (i << 2));
            Eigen::Map<const Eigen::Quaterniond> x(x_ptr + (i << 2));
            const double * delta_ = delta + 3 * i;
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
    
    
    template<int N>
    bool ark::HumanAvatar::MultiQuaternionParameterization<N>::ComputeJacobian(const double* x, double* jacobian) const
    {
        memset(jacobian, 0, 4 * 3 * N*N * sizeof(jacobian[0]));
        double * jacobian_ = jacobian;
        for (int i = 0; i < N; ++i) {
            const double * x_ = x + (i << 2);
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

    template<class T, class VecT_t>
    Eigen::Matrix<T, 3, 1>
        ark::HumanAvatar::_toJointSpace(int joint_id, const VecT_t & vec, T * _pt, T * _cache) const
    {
        return Eigen::Map<Eigen::Matrix<T, 3, 3> >(_cache + joint_id * NUM_ROT_MAT_PARAMS) * vec
            + Eigen::Map<const Eigen::Matrix<T, 3, 1>>(_pt + joint_id * NUM_POS_PARAMS);
    }


    template<class T, class VecT_t>
    Eigen::Matrix<boost::T, 3, 1>
        ark::HumanAvatar::_fromJointSpace(int joint_id, const VecT_t & vec, T * _pt, T * _cache) const
    {
        return Eigen::Map<Eigen::Matrix<T, 3, 3> >(_cache + joint_id * NUM_ROT_MAT_PARAMS).inverse() * (vec
            - Eigen::Map<const Eigen::Matrix<T, 3, 1>>(_pt + joint_id * NUM_POS_PARAMS));
    }

    template<class T>
    void ark::HumanAvatar::_propagateJointTransforms(const T * const _r, const T * const _p, const T * const _w, T * _pb, T * _pt, T * _rt, T * _cache) const
    {
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

            basePos = vec_t(T(0), T(0), T(0));
            for (auto regrEntry : jointRegressor[jid]) {
                vec_t blend = humanPCBase->points[regrEntry.first].getVector3fMap().cast<T>();
                for (size_t j = 0; j < keyNames.size(); ++j) {
                    blend += keyClouds[j].row(regrEntry.first).cast<T>() * _w[j];
                }
                basePos += blend * (T)(regrEntry.second);
            }
        }

        for (size_t jid = 1; jid < joints.size(); ++jid) {
            const Joint::Ptr & joint = joints[jid];
            JointType parID = joint->parent->type;

            const size_t ROT_IDX = jid * NUM_ROT_PARAMS, POS_IDX = jid * NUM_POS_PARAMS;
            quatmap rotTransformed(_rt + ROT_IDX);
            rotTransformed = quatmap(_rt + parID * NUM_ROT_PARAMS) * const_quatmap(_r + ROT_IDX);

            Eigen::Map<Eigen::Matrix<T, 3, 3> >(_cache + NUM_ROT_MAT_PARAMS * jid) = rotTransformed.toRotationMatrix();

            vecmap(_pt + POS_IDX) = _toJointSpace(joint->parent->type,
                vecmap(_pb + POS_IDX) - vecmap(_pb + parID * NUM_POS_PARAMS), _pt, _cache);
        }
    }


    template<class T>
    inline Eigen::Matrix<T, 3, 1> ark::HumanAvatar::_computePointPosition(size_t point_index, const T * const _w, T * _pt, T * _cache) const
    {
        const Point_T & initPt_PCL = humanPCBase->points[point_index];
        typedef Eigen::Matrix<T, 3, 1> vec_t;
        vec_t result = vec_t::Zero(), blend = initPt_PCL.getVector3fMap().cast<T>();

        // add blendshapes
        for (size_t j = 0; j < keyNames.size(); ++j) {
            blend += keyClouds[j].row(point_index).cast<T>() * _w[j];
        }

        // linear blend skinning (LBS)
        for (const auto & bw : boneWeights[point_index]) {
            vec_t initPt = blend - joints[bw.first]->posBase;
            result += _toJointSpace(bw.first, initPt, _pt, _cache) * T(bw.second);
        }
        return result;
    }
}
