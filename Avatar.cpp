#include "stdafx.h"
#include "Visualizer.h"
#include "Util.h"
#include "Avatar.h"

namespace ark {
    HumanAvatar::Joint::Joint(HumanAvatar & avatar, JointType type) :
        avatar(avatar), type(type),
        rotation(avatar._r + NUM_ROT_PARAMS * type),
        scale(avatar._s[NUM_SCALE_PARAMS * type]),
        posBase(avatar._pb + NUM_POS_PARAMS * type),
        posTransformed(avatar._pt + NUM_POS_PARAMS * type),
        rotTransformed(avatar._rt + NUM_ROT_PARAMS * type),
        cachedTransform(avatar._cache + NUM_ROT_MAT_PARAMS * type)
    {
        rotation = Eigen::Quaterniond::Identity();
        scale = 1.0;
        cachedTransform = rotBaseInv = rotBase = Eigen::Matrix3d::Identity();
    }

    HumanAvatar::HumanAvatar(const std::string & model_dir, int downsample_factor) : 
        HumanAvatar(model_dir, std::vector<std::string>(), downsample_factor) { }

    HumanAvatar::HumanAvatar(const std::string & model_dir, const std::vector<std::string> & shape_keys,
        int downsample_factor)
        : MODEL_DIR(model_dir), keyNames(shape_keys), basePos(_p) {

        humanPCBase = boost::unique_ptr<Cloud_T>(new Cloud_T());
        humanPCTransformed = boost::make_shared<Cloud_T>();

        using namespace boost::filesystem;
        path modelPath(model_dir); modelPath = modelPath / "model.pcd";
        path skelPath(model_dir); skelPath = skelPath / "skeleton.txt";

        _w = new double[shape_keys.size()];
        memset(_w, 0, shape_keys.size() * sizeof(double));

        auto humanPCRaw = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		auto humanPCFull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile<pcl::PointXYZ>(modelPath.string(), *humanPCFull);
		//for (int i = 0; i < humanPCFull->points.size(); ++i) {
		//	// skip appropriate number of points if downsample is enabled
		//	if (i % downsample_factor != 0) {
		//		continue;
		//	}
		//	humanPCRaw->push_back(humanPCFull->points[i]);
		//}

		pcl::UniformSampling<pcl::PointXYZ> uniform_downsampler;
		uniform_downsampler.setInputCloud(humanPCFull);
		uniform_downsampler.setRadiusSearch(0.08);
		uniform_downsampler.filter(*humanPCRaw);

		cout << humanPCRaw->size() << endl;
		std::vector<int> introns;
		for (int i = 0; i < humanPCFull->points.size(); ++i) {
			for (int j = 0; j < humanPCRaw->points.size(); ++j) {
				auto full_pt = humanPCFull->points[i];
				auto down_pt = humanPCRaw->points[j];
				if (full_pt.x == down_pt.x && full_pt.y == down_pt.y && full_pt.z == down_pt.z) {
					introns.push_back(i);
				}
			}
		}
		cout << introns.size() << endl;
		cout << humanPCRaw->size() << endl;
		cout << "key cloud" << endl;

        // load all required shape keys
        path keyPath(model_dir); keyPath = keyPath / "shapekey";
        for (std::string k : shape_keys) {
            auto keyPC = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
            pcl::io::loadPCDFile<pcl::PointXYZ>((keyPath / k).string(), *keyPC);
			// Ceiling division to compute the number of points in the down sampled cloud
            //EigenCloud_T keyCloud((keyPC->points.size() + downsample_factor - 1 )/ downsample_factor, 3);
			EigenCloud_T keyCloud(introns.size(), 3);
			int ii = 0;
            for (size_t i = 0; i < keyPC->points.size(); ++i) {
				// skip appropriate number of points if downsample is enabled
				/*if (i % downsample_factor != 0) {
					continue;
				}*/
				if (std::find(introns.begin(), introns.end(), i) == introns.end()) {
					continue;
				}
                keyCloud.row(ii) = keyPC->points[i].getVector3fMap().cast<double>();
				ii++;
            }
            keyClouds.push_back(keyCloud);
			cout << "here" << ii << endl;
        }

        // read skeleton file
        std::ifstream skel(skelPath.string());
        int nJoints, nVerts;
        skel >> nJoints >> nVerts;

        // initialize parameter vectors
        _r = new double[nJoints * NUM_ROT_PARAMS];
        _s = new double[nJoints * NUM_SCALE_PARAMS];
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
            j->posBase = Eigen::Vector3d(x, y, z);
            j->posTransformed = j->posBase;
            if (parID != -1) {
                j->parent = joints[parID].get();
                j->parent->children.push_back(j.get());
            }
            else j->parent = nullptr;
            joints.push_back(j);
        }

		std::cout << nVerts << endl;

        std::vector<double> weights(nJoints);
        //boneWeights.resize((nVerts + downsample_factor - 1) /downsample_factor);
		boneWeights.resize(introns.size());

		int ii = 0;
        // true if the model already provides vertex weights (else we calculate them ourselves)
        bool modelProvidesVertWeights = static_cast<bool>(skel);
        if (modelProvidesVertWeights) {
            for (int i = 0; i < nVerts; ++i) {

                int nEntries; skel >> nEntries;

				// skip appropriate number of points if downsample is enabled
				if (std::find(introns.begin(), introns.end(), i) == introns.end()) {
					for (int j = 0; j < nEntries; ++j) {
						int joint; double w; skel >> joint >> w;
					}
					continue;
				}
				
                std::fill(weights.begin(), weights.end(), 0.0);

                // need to convert vertex (joint) weights to bone weights
                double total = 0.0;
                for (int j = 0; j < nEntries; ++j) {
                    int joint; double w; skel >> joint >> w;
					
                    // for non-leaf joints, add weight to child bones of joint
                    for (Joint * j : joints[joint]->children) {
                        weights[j->type] += w;
                        total += w;
                    }
                    // for leaf joints, add weight to parent bone of joint
                    if (joints[joint]->children.size() == 0) {
                        weights[joint] += w;
                        total += w;
                    }
                }

                // normalize weights to add to 1
                for (int j = 0; j < nJoints; ++j) {
                    if (weights[j] > 0.0) {
						boneWeights[ii].push_back(std::make_pair(j, weights[j] / total));
                    }
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

        for (Joint::Ptr & joint : joints) {
            if (joint->parent == nullptr) continue;
            Eigen::Vector3d vBone = joint->posTransformed - joint->parent->posTransformed;
            Eigen::Quaterniond rot = Eigen::Quaterniond::FromTwoVectors(vBone, Eigen::Vector3d(1, 0, 0));
            joint->rotBase = rot.toRotationMatrix();
            joint->rotBaseInv = rot.inverse().toRotationMatrix();
        }


        // compute locally transformed points
        computeLocalPC();

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
        delete[] _w; delete[] _r; delete[] _s;
        delete[] _pb; delete[] _pt; delete[] _rt;
        delete[] _cache;
    }

    Eigen::Map<Eigen::Vector3d> HumanAvatar::getBasePosition() {
        return basePos;
    }

    Eigen::Vector3d HumanAvatar::getUndeformedBoneVector(JointType joint_id) {
        if (joints[joint_id]->parent == nullptr) return Eigen::Vector3d(0, 0, 0);
        return joints[joint_id]->posBase - joints[joint_id]->parent->posBase;
    }

    Eigen::Vector3d HumanAvatar::getBoneVector(JointType joint_id) {
        if (joints[joint_id]->parent == nullptr) return Eigen::Vector3d(0, 0, 0);
        return joints[joint_id]->posTransformed - joints[joint_id]->parent->posTransformed;
    }

    const Eigen::Map<Eigen::Vector3d> & HumanAvatar::getPosition(JointType joint_id) const {
        return joints[joint_id]->posTransformed;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getLocalRotation(JointType joint_id) {
        return joints[joint_id]->rotation;
    }

    double & HumanAvatar::getLocalScale(JointType joint_id) {
        return joints[joint_id]->scale;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getCenterRotation() {
        return joints[JointType::ROOT]->rotation;
    }

    double & HumanAvatar::getCenterScale() {
        return joints[JointType::ROOT]->scale;
    }

    void HumanAvatar::setCenterPosition(const Eigen::Vector3d & val) {
        basePos = val;
    }

    void HumanAvatar::setCenterScale(double val){
        joints[JointType::ROOT]->scale = val;
    }

    void HumanAvatar::setRotation(HumanAvatar::JointType joint_id, const Eigen::Quaterniond & quat) {
        joints[joint_id]->rotation = quat.normalized();
    }

    /** Set the local rotation of the bone ending at a joint to the given AngleAxis object */
    void HumanAvatar::setRotation(HumanAvatar::JointType joint_id, const Eigen::AngleAxisd & angle_axis) {
        joints[joint_id]->rotation = Eigen::Quaterniond(angle_axis);
    }

    /** Set the local rotation of the bone ending at a joint to the given euler angles */
    void HumanAvatar::setRotation(HumanAvatar::JointType joint_id, double yaw, double pitch, double roll) {
        joints[joint_id]->rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        joints[joint_id]->rotation.normalize();
    }

    /** Set the local rotation of the bone ending at a joint so that v1 in the original space rotates to v2 */
    void HumanAvatar::setRotation(JointType joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2) {
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
    void HumanAvatar::_addRotation(JointType joint_id, Eigen::Vector3d v1, Eigen::Vector3d v2) {
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
    void HumanAvatar::setRotation(JointType joint_id, const Eigen::Vector3d & v) {
        Eigen::Vector3d v1 = getBoneVector(joint_id);
        _addRotation(joint_id, v1, v);
    }


    /** Set the local rotation of the bone ending at a joint so that
      * it points to AND has the length of v */
    void HumanAvatar::setBoneVector(JointType joint_id, const Eigen::Vector3d & v) {
        // rotate to vector
        _addRotation(joint_id, getBoneVector(joint_id), v);

        // find ratio of norms 
        Eigen::Vector3d vu = getUndeformedBoneVector(joint_id);
        double normRatio = v.norm() / vu.norm();

        Joint::Ptr & joint = joints[joint_id];
        joint->scale = normRatio * joints[JointType::ROOT]->scale;
    }

    /** Set the local scale of the bone ending at a joint */
    void HumanAvatar::setScale(JointType joint_id, double scale) {
        joints[joint_id]->scale = scale;
    }

    /** Get a pointer to the specified joint */
    HumanAvatar::Joint::Ptr HumanAvatar::getJoint(JointType joint_id) const {
        return joints[joint_id];
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
            joints[i]->scale = 1.0;
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

            correspondences.clear();
            for (int i = 0; i < dataCloud.rows(); ++i) {
                resultSet.init(&index, &dist);
                mindex.index->findNeighbors(resultSet, dataCloud.data() + i * 3, nanoflann::SearchParams(10));
                correspondences.emplace_back(int(index), i);
            }

            // limit to 1 NN point per model point
            /*
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
                    neighb[i].resize(1);
                    neighb[i][0] = best_idx;
                }
            }
            */
        } else {
            // perform FORWARD NN, i.e. match each model point to a data point
            correspondences.clear();
            std::vector<std::vector<int>> invNN(dataCloud.rows());

            const double * dataPtr = modelCloud.data();
            for (int i = 0; i < modelCloud.rows(); ++i) {
                resultSet.init(&index, &dist);
                mindex->index->findNeighbors(resultSet, dataPtr, nanoflann::SearchParams(10));
                dataPtr += 3;
                invNN[index].push_back(i);
            }

            // limit to 1 NN point per data point
            for (int i = 0; i < dataCloud.rows(); ++i) {
                if (invNN[i].size() > 1) {
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
        static const std::string MODEL_CLOUD_NAME = "model_cloud";
        static const int NN_LINE_INTERVAL = 2;

        if (print_params) {
            // print out model parameters
            double * _w = ava->w(), *_r = ava->r(), *_s = ava->s(), *_p = ava->p();
            for (int i = 0; i < HumanAvatar::NUM_SHAPEKEYS; ++i) {
                cout << _w[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < NUM_JOINTS * HumanAvatar::NUM_ROT_PARAMS; ++i) {
                cout << _r[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < NUM_JOINTS * HumanAvatar::NUM_SCALE_PARAMS; ++i) {
                cout << _s[i] << " ";
            }
            cout << " | ";
            for (int i = 0; i < HumanAvatar::NUM_POS_PARAMS; ++i) {
                cout << _p[i] << " ";
            }
            cout << "\n";
        }

        auto & viewer = Visualizer::getPCLVisualizer();
        viewer->removeAllShapes(1);

        // draw nearest-neighbor1 lines
        for (size_t k = 0; k < correspondences.size(); k += NN_LINE_INTERVAL) {
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
        viewer->removePointCloud(MODEL_CLOUD_NAME, 1);
        viewer->addPointCloud<HumanAvatar::Point_T>(ava->getCloud(), MODEL_CLOUD_NAME, 1);
        viewer->spinOnce();
    }

    void HumanAvatar::fit(const EigenCloud_T & dataCloud) {
        auto startTime = std::chrono::high_resolution_clock::now();
        kd_tree_ptr_t kdTree = _buildKDIndex(dataCloud);
        std::vector<int> joints_subset;
        fitPose(dataCloud, 2, 14, joints_subset, false, kdTree);
        fitShape(dataCloud, 3, 14, false, kdTree);
        fitPose(dataCloud, 4, 14, joints_subset, false, kdTree);
        fitShape(dataCloud, 3, 14, false, kdTree);
        fitPose(dataCloud, 3, 5, joints_subset, false, kdTree);
        auto endTime = std::chrono::high_resolution_clock::now();

        std::cout << "Overall Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << " ms\n";

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        std::vector<std::pair<int, int> > correspondences;
        _updateCloud(_w, _pt, _cache, modelCloud);
        _findNN(kdTree, dataCloud, modelCloud, correspondences, false);
        __debugVisualize(this, dataCloud, modelCloud, correspondences, true);
    }

	void HumanAvatar::fitTrack(const EigenCloud_T & dataCloud) {
		auto startTime = std::chrono::high_resolution_clock::now();
		kd_tree_ptr_t kdTree = _buildKDIndex(dataCloud);
		std::vector<int> joints_subset;
		fitPose(dataCloud, 1, 4, joints_subset, false, kdTree);
		//fitShape(dataCloud, 3, 14, false, kdTree);
		//fitPose(dataCloud, 4, 14, joints_subset, false, kdTree);
		//fitShape(dataCloud, 3, 14, false, kdTree);
		//fitPose(dataCloud, 3, 5, joints_subset, false, kdTree);
		auto endTime = std::chrono::high_resolution_clock::now();

		std::cout << "Overall Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << " ms\n";

		EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
		std::vector<std::pair<int, int> > correspondences;
		_updateCloud(_w, _pt, _cache, modelCloud);
		_findNN(kdTree, dataCloud, modelCloud, correspondences, false);
		__debugVisualize(this, dataCloud, modelCloud, correspondences, true);
	}

    void HumanAvatar::fitPose(const EigenCloud_T & dataCloud, int max_iter, int num_subiter,
        const std::vector<int> & joint_subset, bool inv_nn, kd_tree_ptr_t kd_tree) {
        using namespace ceres;

        kd_tree_ptr_t kdTree = (kd_tree ? kd_tree : _buildKDIndex(dataCloud));

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_pt, _cache, modelCloud);

        EigenCloud_T initJointPos(NUM_JOINTS, 3);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            initJointPos.row(i) = Eigen::Map<Eigen::Vector3d>(_pt + i * NUM_POS_PARAMS);
        }

        auto startTime = std::chrono::high_resolution_clock::now();
        for (int iter = 0; iter < max_iter; ++iter) {
            std::cout << ">> POSE FITTING: ITER " << iter << "\n";
            _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
            _updateCloud(_pt, _cache, modelCloud);

            Problem problem;
            std::vector<std::pair<int, int> > correspondences;

            // find nearest neighbors using nanoflann kd tree
            _findNN(kdTree, dataCloud, modelCloud, correspondences, inv_nn);
            ceres::CostFunction * cost_function =
                new AutoDiffCostFunction<PoseCostFunctor, ceres::DYNAMIC,
                NUM_JOINTS * NUM_ROT_PARAMS,
                NUM_JOINTS * NUM_SCALE_PARAMS,
                NUM_POS_PARAMS>(
                    new PoseCostFunctor(*this, dataCloud, correspondences, initJointPos) ,
                        int(correspondences.size()) * NUM_POS_PARAMS + NUM_JOINTS * NUM_POS_PARAMS);

            problem.AddParameterBlock(_r, NUM_JOINTS * NUM_ROT_PARAMS
                , new MultiQuaternionParameterization<NUM_JOINTS>()
            );
            problem.AddParameterBlock(_s, NUM_JOINTS * NUM_SCALE_PARAMS);
            problem.AddParameterBlock(_p, NUM_POS_PARAMS);
            problem.AddResidualBlock(cost_function, NULL /*new CauchyLoss(25.0)*/,
                _r, _s, _p);

            Solver::Options options;
            options.linear_solver_type = ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            options.initial_trust_region_radius = 200;
            options.minimizer_progress_to_stdout = false;
            options.logging_type = ceres::LoggingType::SILENT;
            options.minimizer_type = ceres::TRUST_REGION;
            options.preconditioner_type = ceres::PreconditionerType::JACOBI;
            //options.line_search_direction_type = ceres::LBFGS;
            options.max_linear_solver_iterations = num_subiter;
            options.max_num_iterations = num_subiter;
            options.num_threads = 1;
            //options.num_linear_solver_threads = 1;
            options.function_tolerance = 1e-5;

            //__debugVisualize(this, dataCloud, modelCloud, neighb);

            // solve ICP
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)
            //std::cout << summary.FullReport() << "\n";

            Visualizer::getPCLVisualizer()->spinOnce();
        }

        auto endTime = std::chrono::high_resolution_clock::now();

        _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
        std::cout << ">> POSE FITTING: DONE\n";
        std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << " ms\n";
    }

    void HumanAvatar::fitShape(const EigenCloud_T & dataCloud, int max_iter, int num_subiter, bool inv_nn, kd_tree_ptr_t kd_tree) {
        using namespace ceres;

        kd_tree_ptr_t kdTree = (kd_tree ? kd_tree : _buildKDIndex(dataCloud));

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);

        auto startTime = std::chrono::high_resolution_clock::now();
        for (int iter = 0; iter < max_iter; ++iter) {
            std::cout << ">> SHAPE FITTING: ITER " << iter << "\n";
            _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
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
                    new ShapeCostFunctor(*this, dataCloud, correspondences), int(correspondences.size()) * NUM_POS_PARAMS);

            problem.AddParameterBlock(_w, NUM_SHAPEKEYS);
            problem.AddResidualBlock(cost_function, NULL /*new CauchyLoss(25.0)*/, _w);

            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
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

            //__debugVisualize(this, dataCloud, modelCloud, neighb);

            // solve ICP
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)
            //std::cout << summary.FullReport() << "\n";

            Visualizer::getPCLVisualizer()->spinOnce();
        }

        auto endTime = std::chrono::high_resolution_clock::now();

        _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
        std::cout << ">> SHAPE FITTING: DONE\n";
        std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << " ms\n";
    }

    void HumanAvatar::alignToJoints(const EigenCloud_T & pos)
    {
        ASSERT(pos.rows() == JointType::_COUNT, "Joint number mismatch");

        Eigen::Vector3d vr = joints[JointType::SPINE1]->posBase - joints[JointType::ROOT]->posBase;
        Eigen::Vector3d vrt = pos.row(JointType::SPINE1) - pos.row(JointType::ROOT);
        basePos = pos.row(0);
        joints[JointType::ROOT]->rotTransformed = joints[JointType::ROOT]->rotation
                                                = Eigen::Quaterniond::FromTwoVectors(vr, vrt);

        double baseScale = (pos.row(JointType::SPINE2) - pos.row(JointType::PELVIS)).norm() /
            (joints[JointType::SPINE2]->posBase - joints[JointType::PELVIS]->posBase).norm();
        joints[JointType::ROOT]->scale = baseScale;

        for (int i = 1; i < pos.rows(); ++i) {
            if (std::isnan(pos.row(i).x()) || std::isnan(pos.row(joints[i]->parent->type).x())) {
                joints[i]->scale = 1.0;
                joints[i]->rotation = Eigen::Quaterniond::Identity();
                joints[i]->rotTransformed = joints[i]->parent->rotTransformed;
                cerr << "B " << i << "; ";
            }
            else {
                Eigen::Vector3d vv = joints[i]->posBase - joints[i]->parent->posBase;
                Eigen::Vector3d vvt = pos.row(i) - pos.row(joints[i]->parent->type);
                std::isnan(pos.row(joints[i]->parent->type).x());
                joints[i]->scale = vvt.norm() / vv.norm() / baseScale;
                joints[i]->rotation = Eigen::Quaterniond::FromTwoVectors(vv, vvt) * joints[i]->parent->rotTransformed.inverse();
                joints[i]->rotTransformed = joints[i]->parent->rotTransformed * joints[i]->rotation;
            }
        }

        propagateJointTransforms();
    }

    void HumanAvatar::visualize(pcl::visualization::PCLVisualizer::Ptr & viewer, std::string pcl_prefix, int viewport) const {
        for (int i = 0; i < joints.size(); ++i) {
            Point_T curr = util::toPCLPoint(joints[i]->posTransformed);
            //std::cerr << "Joint:" << joints[i]->name << ":" << curr.x << "," << curr.y << "," << curr.z << "\n";

            Vec3b color = util::paletteColor(i);
            Vec3f colorf = color / 255.0;
            std::string jointName = pcl_prefix + "avatarJoint" + std::to_string(i);
            viewer->removeShape(jointName, viewport);
            viewer->addSphere(curr, 0.02, colorf[2], colorf[1], colorf[0], jointName, viewport);
            //viewer->addText3D(std::to_string(i), curr, 0.03, 1.0, 1.0, 1.0, jointName + "T", viewport + 1);

            if (joints[i]->parent) {
                Point_T parent = util::toPCLPoint(joints[i]->parent->posTransformed);
                std::string boneName = pcl_prefix + "avatarBone" + std::to_string(i);
                viewer->removeShape(boneName, viewport);
                viewer->addLine(curr, parent, colorf[2], colorf[1], colorf[0], boneName, viewport);
            }
        }
    }

    void HumanAvatar::assignDistanceWeights(int max_vertex_bones, double norm_thresh) {
        const size_t SZ = humanPCTransformed->points.size();
        boneWeights.resize(SZ);
        std::vector<std::pair<double, int>> tmp(joints.size() - 1);
        for (size_t i = 0; i < SZ; ++i) {
            const auto & pt = humanPCTransformed->points[i];
            Vec3f v(pt.x, pt.y, pt.z);
            for (size_t j = 1; j < joints.size(); ++j) {
                if (joints[j]->parent == nullptr) continue;
                const auto & pb = joints[j]->posBase, &ppb = joints[j]->parent->posBase;
                double norm = util::pointLineSegmentNorm(v,
                    Vec3f(pb.x(), pb.y(), pb.z()), Vec3f(ppb.x(), ppb.y(), ppb.z()));
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

    Eigen::Vector3d HumanAvatar::toJointSpace(JointType joint_id, const Eigen::Vector3d & vec) {
        return _toJointSpace(joint_id, vec, _pt, _cache);
    }

    void HumanAvatar::propagateJointTransforms() {
        _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
    }

    void HumanAvatar::computeLocalPC()
    {
        for (int i = 0; i < MAX_ASSIGNED_JOINTS; ++i) {
            localPC[i] = EigenCloud_T(humanPCBase->points.size(), 3);
        }

        for (size_t i = 0; i < humanPCBase->points.size(); ++i) {
            Eigen::Vector3d v = humanPCBase->points[i].getVector3fMap().cast<double>();
            for (size_t j = 0; j < boneWeights[i].size(); ++j) {
                const int jid = boneWeights[i][j].first;
                const auto & jnt = joints[jid];
                localPC[j].row(i) = jnt->rotBaseInv * (v - jnt->parent->posBase);
            }
        }
    }

    Eigen::Vector3d HumanAvatar::computePointPosition(size_t point_index) {
        return _computePointPosition(point_index, _w, _pt, _cache);
    }
}
