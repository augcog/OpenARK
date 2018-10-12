#include "stdafx.h"
#include "Visualizer.h"
#include "Util.h"
#include "Avatar.h"

namespace ark {
    HumanAvatar::Joint::Joint(HumanAvatar & avatar, JointType type) :
        avatar(avatar), type(type),
        rotation(avatar._r + NUM_ROT_PARAMS * type),
        scale(avatar._s + NUM_SCALE_PARAMS * type),
        posBase(avatar._pb + NUM_POS_PARAMS * type),
        posTransformed(avatar._pt + NUM_POS_PARAMS * type),
        rotTransformed(avatar._rt + NUM_ROT_PARAMS * type),
        cachedTransform(avatar._cache + NUM_ROT_MAT_PARAMS * type)
    {
        rotation = Eigen::Quaterniond(1, 0, 0, 0);
        scale = Eigen::Vector3d(1, 1, 1);
        cachedTransform = rotBaseInv = rotBase = Eigen::Matrix3d::Identity();
    }

    HumanAvatar::HumanAvatar(const std::string & model_dir) : HumanAvatar(model_dir, std::vector<std::string>()) { }

    HumanAvatar::HumanAvatar(const std::string & model_dir, const std::vector<std::string> & shape_keys, const int downsample_factor)
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
		for (int i = 0; i < humanPCFull->points.size(); ++i) {
			// skip appropriate number of points if downsample is enabled
			if (i % downsample_factor != 0) {
				continue;
			}
			humanPCRaw->push_back(humanPCFull->points[i]);
		}

        // load all required shape keys
        path keyPath(model_dir); keyPath = keyPath / "shapekey";
        for (std::string k : shape_keys) {
            auto keyPC = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
            pcl::io::loadPCDFile<pcl::PointXYZ>((keyPath / k).string(), *keyPC);
			// Ceiling division to compute the number of points in the down sampled cloud
            EigenCloud_T keyCloud((keyPC->points.size() + downsample_factor - 1 )/ downsample_factor, 3);
			int ii = 0;
            for (size_t i = 0; i < keyPC->points.size(); ++i) {
				// skip appropriate number of points if downsample is enabled
				if (i % downsample_factor != 0) {
					continue;
				}
                keyCloud.row(ii) = keyPC->points[i].getVector3fMap().cast<double>();
				ii++;
            }
            keyClouds.push_back(keyCloud);
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

        std::vector<double> weights(nJoints);
        boneWeights.resize((nVerts + downsample_factor - 1) /downsample_factor);

		int ii = 0;
        // true if the model already provides vertex weights (else we calculate them ourselves)
        bool modelProvidesVertWeights = static_cast<bool>(skel);
        if (modelProvidesVertWeights) {
            for (int i = 0; i < nVerts; ++i) {

                int nEntries; skel >> nEntries;

				// skip appropriate number of points if downsample is enabled
				if (i % downsample_factor != 0) {
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
            vBone.normalize();
            double dot = vBone.dot(Eigen::Vector3d(1, 0, 0));
            if (dot <= 0.9999 && dot >= -0.9999) {
                Eigen::AngleAxisd rot(acosf(dot), vBone.cross(Eigen::Vector3d(1, 0, 0)));
                joint->rotBase = rot.toRotationMatrix();
                joint->rotBaseInv = rot.inverse().toRotationMatrix();
            }
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

    Eigen::Map<Eigen::Vector3d> & HumanAvatar::getLocalScale(JointType joint_id) {
        return joints[joint_id]->scale;
    }

    Eigen::Map<Eigen::Quaterniond> & HumanAvatar::getBaseRotation() {
        return joints[JointType::ROOT]->rotation;
    }

    Eigen::Map<Eigen::Vector3d> & HumanAvatar::getBaseScale() {
        return joints[JointType::ROOT]->scale;
    }

    void HumanAvatar::setCenterPosition(const Eigen::Vector3d & val) {
        basePos = val;
    }

    void HumanAvatar::setCenterScale(const Eigen::Vector3d & val){
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
        joint->scale = Eigen::Vector3d(normRatio, 1, 1).cwiseQuotient(joints[JointType::ROOT]->scale);
    }

    /** Set the local scale of the bone ending at a joint */
    void HumanAvatar::setScale(JointType joint_id, const Eigen::Vector3d & scale) {
        joints[joint_id]->scale = scale;
    }

    /** Set the local scale of the bone ending at a joint */
    void HumanAvatar::setScale(JointType joint_id, double x, double y, double z) {
        joints[joint_id]->scale = Eigen::Vector3d(x, y, z);
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
            joints[i]->scale = Eigen::Vector3d(1, 1, 1);
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

    void HumanAvatar::_findNN(const EigenCloud_T & dataCloud, const EigenCloud_T & modelCloud,
        std::vector<std::vector<int>> & neighb) {

        using namespace nanoflann;
        typedef KDTreeEigenMatrixAdaptor<EigenCloud_T, 3, metric_L2_Simple>  kd_tree_t;
        kd_tree_t mindex(3, modelCloud, 10);
        mindex.index->buildIndex();

        size_t index; double dist;
        nanoflann::KNNResultSet<double> resultSet(1);

        neighb.clear(); neighb.resize(modelCloud.rows());

        for (int i = 0; i < dataCloud.rows(); ++i) {
            resultSet.init(&index, &dist);
            mindex.index->findNeighbors(resultSet, dataCloud.data() + i * 3, nanoflann::SearchParams(10));
            neighb[index].push_back(i);
        }
    }

    // function called at each iteration of optimization procedure, used for debugging
    static void __debugVisualize(HumanAvatar * ava, const HumanAvatar::EigenCloud_T & dataCloud,
        const HumanAvatar::EigenCloud_T & modelCloud, const std::vector<std::vector<int>> & neighb, bool print_params = false) {

        const int NUM_JOINTS = ava->numJoints();
        static const std::string MODEL_CLOUD_NAME = "model_cloud";
        static const int NN_LINE_INTERVAL = 5;

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
        viewer->removeAllShapes();

        // draw nearest-neighbor lines
        int k = 0;
        for (int i = 0; i < modelCloud.rows(); ++i) {
            HumanAvatar::Point_T p1; p1.getVector3fMap() = modelCloud.row(i).cast<float>();
            for (int j : neighb[i]) {
                ++k;
                if (k % NN_LINE_INTERVAL == 0) {
                    HumanAvatar::Point_T p2; p2.getVector3fMap() = dataCloud.row(j).cast<float>();
                    std::string name = "nn_line_" + std::to_string(j);
                    viewer->addLine<HumanAvatar::Point_T, HumanAvatar::Point_T>(p2, p1, 1.0, 0.0, 0.0, name, 0);
                }
            }
        }

        // re-draw model joints and points
        ava->update(false);
        ava->visualize(viewer, "ava_");
        viewer->removePointCloud(MODEL_CLOUD_NAME);
        viewer->addPointCloud<HumanAvatar::Point_T>(ava->getCloud(), MODEL_CLOUD_NAME, 0);
        viewer->spinOnce();
    }

    void HumanAvatar::_alignICP(const EigenCloud_T & dataCloud, EigenCloud_T & modelCloud, int n_iter) {
        using namespace ceres;

        // ceres will run for 'CERES_ITER_INCR' more iterations at each ICP iteration
        const int CERES_ITER_INCR = 1;

        Problem problem;
        std::vector<std::vector<int> > neighb;
        ceres::CostFunction * cost_function =
            new AutoDiffCostFunction<AlignICPCostFunctor, 1,
                NUM_ROT_PARAMS,
                NUM_POS_PARAMS>(
                    new AlignICPCostFunctor(*this, dataCloud, neighb));

        problem.AddParameterBlock(_r, NUM_ROT_PARAMS, new EigenQuaternionParameterization());
        problem.AddParameterBlock(_p, NUM_POS_PARAMS);
        problem.AddResidualBlock(cost_function, NULL, _r, _p);

        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.logging_type = ceres::SILENT;
        options.minimizer_type = ceres::LINE_SEARCH;
        options.max_linear_solver_iterations = 4;
        options.max_num_iterations = 4;
        options.num_threads = 4;
        options.function_tolerance = 2e-5;

        for (int iter = 0; iter < n_iter; ++iter) {
            std::cout << ">> ALIGNMENT ICP: ITER " << iter << "\n";
            _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
            _updateCloud(_w, _pt, _cache, modelCloud);

            // find nearest neighbors using nanoflann kd tree
            _findNN(dataCloud, modelCloud, neighb);
            //__debugVisualize(this, dataCloud, modelCloud, neighb);

            // solve ICP
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)
            //std::cout << summary.BriefReport() << "\n";

            Visualizer::getPCLVisualizer()->spinOnce();
            options.max_num_iterations = options.max_linear_solver_iterations += CERES_ITER_INCR;
        }

        _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
        _updateCloud(_w, _pt, _cache, modelCloud);
        _findNN(dataCloud, modelCloud, neighb);
        //__debugVisualize(this, dataCloud, modelCloud, neighb);
        std::cout << ">> ALIGNMENT ICP: DONE\n";
    }

    void HumanAvatar::alignICP(const EigenCloud_T & dataCloud, int n_iter) {
        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);
        _alignICP(dataCloud, modelCloud);
    }

    void HumanAvatar::fit(const EigenCloud_T & dataCloud) {
        using namespace ceres;

        EigenCloud_T modelCloud(humanPCTransformed->points.size(), 3);
        _updateCloud(_w, _pt, _cache, modelCloud);

        _alignICP(dataCloud, modelCloud);

        // max optimization iterations
        const int MAX_SOLVE_ITER = 6;
        // ceres will run for 'CERES_ITER_INCR' more iterations on each successive ICP iteration
        const int CERES_ITER_INCR = 2;
        // re-aligns using ICP every 'ALIGN_INTERVAL' iterations.
        //const int ALIGN_INTERVAL = 3;

        Problem problem;
        std::vector<std::vector<int> > neighb;
        ceres::CostFunction * cost_function =
            new AutoDiffCostFunction<AvatarFitCostFunctor, 1,
                NUM_SHAPEKEYS,
                NUM_JOINTS * NUM_ROT_PARAMS,
                NUM_JOINTS * NUM_SCALE_PARAMS,
                NUM_POS_PARAMS>(
                    new AvatarFitCostFunctor(*this, dataCloud, neighb));

        problem.AddParameterBlock(_w, NUM_SHAPEKEYS );
        problem.AddParameterBlock(_r, NUM_JOINTS * NUM_ROT_PARAMS
            , new MultiQuaternionParameterization<NUM_JOINTS>()
        );
        problem.AddParameterBlock(_s, NUM_JOINTS * NUM_SCALE_PARAMS);
        problem.AddParameterBlock(_p, NUM_POS_PARAMS);
        problem.AddResidualBlock(cost_function, new CauchyLoss(25.0),
                                 _w, _r, _s, _p);

        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        options.minimizer_type = ceres::LINE_SEARCH;
        options.max_linear_solver_iterations = 5;
        options.max_num_iterations = 5;
        options.num_threads = 4;
        options.function_tolerance = 2e-5;

        for (int iter = 0; iter < MAX_SOLVE_ITER; ++iter) {
            std::cout << ">> FIT: ITER " << iter << "\n";
            _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
            _updateCloud(_w, _pt, _cache, modelCloud);

            // find nearest neighbors using nanoflann kd tree
            _findNN(dataCloud, modelCloud, neighb);
            __debugVisualize(this, dataCloud, modelCloud, neighb);

            // solve ICP
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)
            std::cout << summary.BriefReport() << "\n";

            Visualizer::getPCLVisualizer()->spinOnce();
            options.max_num_iterations = options.max_linear_solver_iterations += CERES_ITER_INCR;
            
            // re-align
            //if (iter % ALIGN_INTERVAL == ALIGN_INTERVAL - 1) _alignICP(dataCloud, modelCloud, 1);
        }

        _propagateJointTransforms(_r, _s, _p, _pt, _rt, _cache);
        _updateCloud(_w, _pt, _cache, modelCloud);
        _findNN(dataCloud, modelCloud, neighb);
        __debugVisualize(this, dataCloud, modelCloud, neighb, true);
        std::cout << ">> FIT: DONE\n";
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
