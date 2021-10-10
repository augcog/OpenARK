#include "stdafx.h"
#include "Visualizer.h"
#include "Util.h"
#include "Avatar.h"
#include "HumanDetector.h"

namespace {
    typedef ark::HumanAvatar::EigenCloud_T cloud;
    typedef ark::HumanAvatar::JointType smpl_j;
    typedef ark::HumanDetector::OpenPoseMPIJoint mpi_j;
}

namespace ark {
    /** UKF Model for HumanAvatar */
    struct HumanAvatarUKFModel {
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
        static void init(kalman::UKF<HumanAvatarUKFModel, HumanAvatar> & ukf) {
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
        static StateVec dF(const StateVec & state, const HumanAvatar & input) {
            StateVec out = state;
            ark::kalman::util::diffQuaternion(out, 0);
            ark::kalman::util::diffPosition(out, StateVec::QUAT_START - 6,
                StateVec::QUAT_START - 3, StateVec::QUAT_START - 3);
            return out;
        }

        /* measurement model definition */
        static MeasureVec H(const StateVec & state, const HumanAvatar & input) {

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

    const std::pair<int, int> HumanAvatar::MATCHED_JOINTS[] = {
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
        : MODEL_DIR(model_dir), keyNames(shape_keys), basePos(_p) {

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

    HumanAvatar::~HumanAvatar() {
        // clean up parameter vectors
        delete[] _w; delete[] _r; delete[] _rr;
        delete[] _pb; delete[] _pt; delete[] _rt;
        delete[] _cache;
    }

    Eigen::VectorXd HumanAvatar::smplParams() const
    {
        return _smplParams(_r);
    }

    Eigen::Map<Eigen::Vector3d> HumanAvatar::getBasePosition() {
        return basePos;
    }

    Eigen::Vector3d HumanAvatar::getUndeformedBoneVector(int joint_id) {
        if (joints[joint_id]->parent == nullptr) return Eigen::Vector3d(0, 0, 0);
        return joints[joint_id]->posBase - joints[joint_id]->parent->posBase;
    }

    Eigen::Vector3d HumanAvatar::getBoneVector(int joint_id) {
        if (joints[joint_id]->parent == nullptr) return Eigen::Vector3d(0, 0, 0);
        return joints[joint_id]->posTransformed - joints[joint_id]->parent->posTransformed;
    }

    const Eigen::Map<Eigen::Vector3d> & HumanAvatar::getPosition(int joint_id) const {
        return joints[joint_id]->posTransformed;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getLocalRotation(int joint_id) {
        return joints[joint_id]->rotation;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getCenterRotation() {
        return joints[JointType::ROOT_PELVIS]->rotation;
    }

    void HumanAvatar::setCenterPosition(const Eigen::Vector3d & val) {
        basePos = val;
    }

    void HumanAvatar::setRotation(int joint_id, const Eigen::Quaterniond & quat) {
        joints[joint_id]->rotation = quat.normalized();
    }

    /** Set the local rotation of the bone ending at a joint to the given AngleAxis object */
    void HumanAvatar::setRotation(int joint_id, const Eigen::AngleAxisd & angle_axis) {
        joints[joint_id]->rotation = Eigen::Quaterniond(angle_axis);
    }

    /** Set the local rotation of the bone ending at a joint to the given euler angles */
    void HumanAvatar::setRotation(int joint_id, double yaw, double pitch, double roll) {
        joints[joint_id]->rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        joints[joint_id]->rotation.normalize();
    }

    /** Set the local rotation of the bone ending at a joint so that v1 in the original space rotates to v2 */
    void HumanAvatar::setRotation(int joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2) {
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
    void HumanAvatar::_addRotation(int joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2) {
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
    void HumanAvatar::setRotation(int joint_id, const Eigen::Vector3d & v) {
        Eigen::Vector3d v1 = getBoneVector(joint_id);
        _addRotation(joint_id, v1, v);
    }

    /** Get a pointer to the specified joint */
    HumanAvatar::Joint::Ptr HumanAvatar::getJoint(int joint_id) const {
        return joints[joint_id];
    }

    Eigen::Vector3d HumanAvatar::getJointPosition(int joint_id) const {
        return joints[joint_id]->posTransformed;
    }

    Eigen::Vector2d HumanAvatar::getJointPosition2d(int joint_id) const {
        return HumanDetector::projectToImage(pinholeIntrin, joints[joint_id]->posTransformed);
    }

    /** Get the number of joints in the avatar's skeleton */
    int HumanAvatar::numJoints() const {
        return (int)joints.size();
    }

    /** Get the weight of the given shape key (blendshape) */
    double & HumanAvatar::getKeyWeight(int id) {
        return _w[id];
    }

    /** Get the name of the given shape key (blendshape) */
    const std::string & HumanAvatar::getKeyName(int id) const {
        return keyNames[id];
    }

    /** Set the weight of the given shape key (blendshape) */
    void HumanAvatar::setKeyWeight(int id, double weight) {
        _w[id] = weight;
    }

    /** Get the number of shape keys (blendshapes) available */
    int HumanAvatar::numKeys() const {
        return (int)keyNames.size();
    }

    void HumanAvatar::reset(bool update) {
        memset(_p, 0, NUM_POS_PARAMS * sizeof(_p[0]));
        memset(_w, 0, NUM_SHAPEKEYS * sizeof(_w[0]));
        for (int i = 0; i < joints.size(); ++i) {
            joints[i]->rotation = Eigen::Quaterniond::Identity();
        }
        if (update) {
            this->update();
        }
    }

    HumanAvatar::Cloud_T::Ptr HumanAvatar::getCloud(bool update) {
        if (update) this->update();
        return humanPCTransformed;
    }

    void HumanAvatar::update(bool propagate) {
        if (propagate) propagateJointTransforms();

        for (size_t i = 0; i < humanPCTransformed->points.size(); ++i) {
            humanPCTransformed->points[i].getVector3fMap() = computePointPosition(i).cast<float>();
        }
    }

    void HumanAvatar::colorByWeights() {
        for (int i = 0; i < (int)humanPCTransformed->points.size(); ++i) {
            humanPCTransformed->points[i].rgb = 0;
            // color based on weights
            for (int j = 0; j < (int)boneWeights[i].size(); ++j) {
                Vec3b color = util::paletteColor(boneWeights[i][j].first, false);
                humanPCTransformed->points[i].r += color[0] * boneWeights[i][j].second;
                humanPCTransformed->points[i].g += color[1] * boneWeights[i][j].second;
                humanPCTransformed->points[i].b += color[2] * boneWeights[i][j].second;
            }
        }
    }

    void HumanAvatar::color(std::vector<std::vector<int>> & groups) {
        for (int g = 0; g < groups.size(); ++g) {
            Vec3b color = util::paletteColor(g, false);
            for (int i : groups[g]) {
                humanPCTransformed->points[i].r = color[0];
                humanPCTransformed->points[i].g = color[1];
                humanPCTransformed->points[i].b = color[2];
            }
        }
    }

    HumanAvatar::kd_tree_ptr_t HumanAvatar::_buildKDIndex(const EigenCloud_T & dataCloud){
        using namespace nanoflann;
        auto out = std::make_shared<kd_tree_t>(3, dataCloud, 10);
        out->index->buildIndex();
        return out;
    }

    void HumanAvatar::_findNN(const kd_tree_ptr_t & mindex, const EigenCloud_T & dataCloud, const EigenCloud_T & modelCloud,
        std::vector<std::pair<int, int>> & correspondences, bool inverted) {

        size_t index; double dist;
        nanoflann::KNNResultSet<double> resultSet(1);
        
        if (inverted) {
            // perform INVERSE NN, i.e. match each data point to a model point
            kd_tree_t mindex(3, modelCloud, 10);
            mindex.index->buildIndex();

            std::vector<std::vector<int>> neighb(modelCloud.rows());

            correspondences.clear();
            for (int i = 0; i < dataCloud.rows(); ++i) {
                resultSet.init(&index, &dist);
                mindex.index->findNeighbors(resultSet, dataCloud.data() + i * 3, nanoflann::SearchParams(10));
                neighb[int(index)].emplace_back(i);
            }

            // limit to 1 NN point per model point
            for (int i = 0; i < modelCloud.rows(); ++i) {
                int best_idx = -1; double best_norm = DBL_MAX;
                for (int nei : neighb[i]) {
                    double norm = (dataCloud.row(nei) - modelCloud.row(i)).squaredNorm();
                    if (norm < best_norm) {
                        best_norm = norm;
                        best_idx = nei;
                    }
                }
                if (~best_idx) {
                    correspondences.emplace_back(i, best_idx);
                }
            }
        } else {
            // perform FORWARD NN, i.e. match each model point to a data point
            correspondences.clear();
            std::vector<std::vector<int>> invNN(dataCloud.rows());

            const double * dataPtr = modelCloud.data();
            for (int i = 0; i < modelCloud.rows(); ++i) {
                resultSet.init(&index, &dist);
                mindex->index->findNeighbors(resultSet, dataPtr, nanoflann::SearchParams(10));
                dataPtr += 3;
                if (index >= 0 && index < static_cast<int>(invNN.size()))
                    invNN[index].push_back(i);
            }

            // limit to 1 NN point per data point
            for (int i = 0; i < dataCloud.rows(); ++i) {
                if (invNN[i].size() > 0) {
                    int best_idx = 0; double best_norm = DBL_MAX;
                    for (int j = 0; j < invNN[i].size(); ++j) {
                        double norm = (dataCloud.row(i) - modelCloud.row(invNN[i][j])).squaredNorm();
                        if (norm < best_norm) {
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
                                                      bool print_params = false) {

        const int NUM_JOINTS = ava->numJoints();

        if (print_params) {
            // print out model parameters
            double * _w = ava->w(), *_r = ava->r(), *_p = ava->p();
            for (int i = 0; i < HumanAvatar::NUM_SHAPEKEYS; ++i) {
                cout << _w[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < NUM_JOINTS * HumanAvatar::NUM_ROT_PARAMS; ++i) {
                cout << _r[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < HumanAvatar::NUM_POS_PARAMS; ++i) {
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

    void HumanAvatar::fit(const EigenCloud_T & dataCloud, double deltat, bool track) {
        static kalman::UKF<HumanAvatarUKFModel, HumanAvatar> ukf;

        auto startTime = std::chrono::high_resolution_clock::now();
        kd_tree_ptr_t kdTree = _buildKDIndex(dataCloud);
        std::vector<int> joints_subset;
        if (!track) {
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
        else {
            fitPose(dataCloud, 2, 4, joints_subset, true, kdTree);

            // update ukf
            Eigen::Map<Eigen::Matrix<double, NUM_JOINTS*3, 1>> z(_pt);
            if (deltat < 0) {
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
        const std::vector<int> & joint_subset, bool inv_nn, kd_tree_ptr_t kd_tree) {
        using namespace ceres;

        kd_tree_ptr_t kdTree = (kd_tree ? kd_tree : _buildKDIndex(dataCloud));

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);

        EigenCloud_T initJointPos(NUM_JOINTS, 3);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            initJointPos.row(i) = Eigen::Map<Eigen::Vector3d>(_pt + i * NUM_POS_PARAMS);
        }

        for (int iter = 0; iter < max_iter; ++iter) {
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

    void HumanAvatar::fitShape(const EigenCloud_T & dataCloud, int max_iter, int num_subiter, bool inv_nn, kd_tree_ptr_t kd_tree) {
        using namespace ceres;

        kd_tree_ptr_t kdTree = (kd_tree ? kd_tree : _buildKDIndex(dataCloud));

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);

        for (int iter = 0; iter < max_iter; ++iter) {
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
        for (int i = 1; i < pos.rows(); ++i) {
            scaleAvg += (pos.row(i) - pos.row(joints[i]->parent->type)).norm() /
                (joints[i]->posSkel - joints[i]->parent->posSkel).norm();
        }
        scaleAvg /= (pos.rows() - 1.0);
        double baseScale = (joints[JointType::SPINE2]->posSkel - joints[JointType::ROOT_PELVIS]->posSkel).norm() * (scaleAvg - 1.0);
        _w[0] = baseScale * PC1_DIST_FACT;
        if (isnan(_w[0])) _w[0] = 1.5;
        propagateJointTransforms();

        for (int i = 1; i < pos.rows(); ++i) {
            joints[i]->rotation = Eigen::Quaterniond::Identity();
            joints[i]->rotTransformed = joints[i]->parent->rotTransformed;
            joints[i]->cachedTransform = joints[i]->rotTransformed.toRotationMatrix();
            if (joints[i]->children.empty() || std::isnan(pos.row(i).x()) || std::isnan(pos.row(joints[i]->children[0]->type).x())) {
            }
            else {
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

    void HumanAvatar::visualize(const pcl::visualization::PCLVisualizer::Ptr & viewer, std::string pcl_prefix, int viewport) const {
        for (int i = 0; i < joints.size(); ++i) {
            Point_T curr = util::toPCLPoint(joints[i]->posTransformed);
            //std::cerr << "Joint:" << joints[i]->name << ":" << curr.x << "," << curr.y << "," << curr.z << "\n";

            Vec3b color = util::paletteColor(i);
            Vec3f colorf = color / 255.0;
            std::string jointName = pcl_prefix + "avatarJoint" + std::to_string(i);
            viewer->removeShape(jointName, viewport);
            viewer->addSphere(curr, 0.02, colorf[2], colorf[1], colorf[0], jointName, viewport);

            if (joints[i]->parent) {
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

    void HumanAvatar::assignDistanceWeights(int max_vertex_bones, double norm_thresh) {
        const size_t SZ = humanPCTransformed->points.size();
        boneWeights.resize(SZ);
        std::vector<std::pair<double, int>> tmp(joints.size() - 1);
        for (size_t i = 0; i < SZ; ++i) {
            const auto & pt = humanPCTransformed->points[i];
            Eigen::Vector3d v(pt.x, pt.y, pt.z);
            for (size_t j = 1; j < joints.size(); ++j) {
                if (joints[j]->parent == nullptr) continue;
                const auto & pb = joints[j]->posBase;
                double norm = (v - pb).norm();
                tmp[j - 1].first = norm;
                tmp[j - 1].second = (int)j;
            }
            std::sort(tmp.begin(), tmp.end());
            double totalWt = 0.0;
            for (int j = 0; j < max_vertex_bones; ++j) {
                if (j && tmp[j].first > norm_thresh) break;
                boneWeights[i].push_back(std::make_pair(tmp[j].second,
                    1.0 / (tmp[j].first * tmp[j].first)
                ));
                totalWt += boneWeights[i][j].second;
            }

            for (size_t j = 0; j < boneWeights[i].size(); ++j) {
                boneWeights[i][j].second /= totalWt;
            }
        }
    }

    Eigen::Vector3d HumanAvatar::toJointSpace(int joint_id, const Eigen::Vector3d & vec) {
        return _toJointSpace(joint_id, vec, _pt, _cache);
    }

    void HumanAvatar::propagateJointTransforms() {
        _propagateJointTransforms(_r, _p, _w, _pb, _pt, _rt, _cache);
    }

    Eigen::Vector3d HumanAvatar::computePointPosition(size_t point_index) {
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
        for (int i = 0; i < nComps; ++i) {
            // load weights
            ifs >> weight[i];
            consts_log[i] = log(weight[i]) - log_sqrt_2_pi_n;
            consts[i] = weight[i] / sqrt_2_pi_n;
        }

        mean.resize(nComps, nDims);
        for (int i = 0; i < nComps; ++i) {
            for (int j = 0; j < nDims; ++j) {
                // load mean vectors
                ifs >> mean(i, j);
            }
        }

        /** Cholesky decomposition */
        typedef Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> Cholesky;

        cov.resize(nComps);
        cov_cho.resize(nComps);
        double maxDet = 0.0;
        for (int i = 0; i < nComps; ++i) {
            auto & m = cov[i];
            m.resize(nDims, nDims);
            for (int j = 0; j < nDims; ++j) {
                for (int k = 0; k < nDims; ++k) {
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

        for (int i = 0; i < nComps; ++i) {
            // normalize constants
            consts[i] /= maxDet;
            consts_log[i] -= log(maxDet);
        }
    }
}
