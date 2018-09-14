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
        rotBase(avatar._rb + NUM_ROT_PARAMS * type),
        posTransformed(avatar._pt + NUM_POS_PARAMS * type),
        rotTransformed(avatar._rt + NUM_ROT_PARAMS * type)
    {
        rotation = Eigen::Quaterniond(1, 0, 0, 0);
        scale = Eigen::Vector3d(1, 1, 1);
        rotBase = Eigen::Quaterniond::Identity();
    }

    HumanAvatar::HumanAvatar(const std::string & model_dir) : HumanAvatar(model_dir, std::vector<std::string>()) { }

    HumanAvatar::HumanAvatar(const std::string & model_dir, const std::vector<std::string> & shape_keys)
        : MODEL_DIR(model_dir), keyNames(shape_keys), basePos(_p) {

        humanPCBase = boost::unique_ptr<Cloud_T>(new Cloud_T());
        humanPCTransformed = boost::make_shared<Cloud_T>();

        using namespace boost::filesystem;
        path modelPath(model_dir); modelPath = modelPath / "model.pcd";
        path skelPath(model_dir); skelPath = skelPath / "skeleton.txt";

        _w = new double[shape_keys.size()];
        memset(_w, 0, shape_keys.size() * sizeof(double));

        auto humanPCRaw = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile<pcl::PointXYZ>(modelPath.string(), *humanPCRaw);

        // load all required shape keys
        path keyPath(model_dir); keyPath = keyPath / "shapekey";
        for (std::string k : shape_keys) {
            auto keyPC = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
            pcl::io::loadPCDFile<pcl::PointXYZ>((keyPath / k).string(), *keyPC);
            keyClouds.push_back(keyPC);
        }

        // read skeleton file
        std::ifstream skel(skelPath.string());
        int nJoints, nVerts;
        skel >> nJoints >> nVerts;

        // initialize parameter vectors
        _r = new double[nJoints * NUM_ROT_PARAMS];
        _s = new double[nJoints * NUM_SCALE_PARAMS];
        _pb = new double[nJoints * NUM_POS_PARAMS];
        _rb = new double[nJoints * NUM_ROT_PARAMS];
        _pt = new double[nJoints * NUM_POS_PARAMS];
        _rt = new double[nJoints * NUM_ROT_PARAMS];
        memset(_rb, 0, nJoints * NUM_ROT_PARAMS * sizeof(_rb[0]));

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
        boneWeights.resize(nVerts);

        // true if the model already provides vertex weights (else we calculate them ourselves)
        bool modelProvidesVertWeights = static_cast<bool>(skel);
        if (modelProvidesVertWeights) {
            for (int i = 0; i < nVerts; ++i) {
                int nEntries; skel >> nEntries;

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
                        boneWeights[i].push_back(std::make_pair(j, weights[j] / total));
                    }
                }
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
                joint->rotBase = Eigen::AngleAxisd(acosf(dot), vBone.cross(Eigen::Vector3d(1, 0, 0)));
            }
        }

        if (!modelProvidesVertWeights) {
            std::cerr << "WARNING: no vertex weights found in avatar model." <<
                "Initializing based on distance...\n";
            // assign skeleton weights using distance metric
            assignDistanceWeights();
        }

        // color the point cloud
        for (int i = 0; i < (int)humanPCTransformed->points.size(); ++i) {
            humanPCTransformed->points[i].rgb = 0;
            // color based on weights
            for (int j = 0; j < (int)boneWeights[i].size(); ++j) {
                Vec3b color = util::paletteColor(boneWeights[i][j].first, false);
                humanPCTransformed->points[i].r += color[2] * boneWeights[i][j].second;
                humanPCTransformed->points[i].g += color[1] * boneWeights[i][j].second;
                humanPCTransformed->points[i].b += color[0] * boneWeights[i][j].second;
            }
        }
    }

    HumanAvatar::~HumanAvatar() {
        // clean up parameter vectors
        delete[] _w; delete[] _r; delete[] _s;
        delete[] _pb; delete[] _rb;
        delete[] _pt; delete[] _rt;
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

    void HumanAvatar::_findNN(const EigenCloud_T & dataCloud, const EigenCloud_T & modelCloud,
        std::vector<std::vector<int>> & closest) {

        using namespace nanoflann;
        typedef KDTreeEigenMatrixAdaptor<EigenCloud_T, 3, metric_L2_Simple>  kd_tree_t;
        kd_tree_t mindex(3, modelCloud, 10);
        mindex.index->buildIndex();

        size_t index; double dist;
        nanoflann::KNNResultSet<double> resultSet(1);

        closest.clear(); closest.resize(modelCloud.rows());

        for (int i = 0; i < dataCloud.rows(); ++i) {
            resultSet.init(&index, &dist);
            mindex.index->findNeighbors(resultSet, dataCloud.data() + i * 3, nanoflann::SearchParams(10));
            closest[index].push_back(i);
        }
    }

    void HumanAvatar::fitShapeTo(const Cloud_T::Ptr & cloud) {
        using namespace ceres;

        // store point cloud in Eigen format
        EigenCloud_T dataCloud(cloud->points.size(), 3);
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            dataCloud.row(i) = cloud->points[i].getVector3fMap().cast<double>();
        }

        EigenCloud_T modelCloud;
        _updateCloud(_w, _pt, _rt, _s, modelCloud);

        // Hard-coded weight key, joint numbers. This allows us to use static cost function with Ceres.
        const int NUM_WKEY = 10, NUM_JOINTS = HumanAvatar::JointType::_COUNT;

        // Max optimization iterations
        const int MAX_ITER = 20;

        Problem problem;
        std::vector<std::vector<int> > closest;
        CostFunction * cost_function =
            new AutoDiffCostFunction<HumanAvatarCostFunctor, NUM_WKEY,
            NUM_JOINTS * NUM_ROT_PARAMS,
            NUM_JOINTS * NUM_SCALE_PARAMS,
            NUM_POS_PARAMS, 1>(
                new HumanAvatarCostFunctor(*this, dataCloud, closest));

        problem.AddResidualBlock(cost_function, NULL, _w, _r, _s, _p);

        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // find nearest neighbors using nanoflann kd tree
            _findNN(dataCloud, modelCloud, closest);

            // solve ICP
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // output (for debugging)
            std::cout << summary.BriefReport() << "\n";
        }

        // update to show fitted parameters
        update();
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
        return _toJointSpace(joint_id, vec, _pt, _rt, _s);
    }

    void HumanAvatar::propagateJointTransforms() {
        _propagateJointTransforms(_r, _s, _p, _pt, _rt);
    }

    Eigen::Vector3d HumanAvatar::computePointPosition(size_t point_index) {
        return _computePointPosition(point_index, _w, _pt, _rt, _s);
    }
}
