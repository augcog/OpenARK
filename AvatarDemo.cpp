#include "stdafx.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <opencv2/ximgproc.hpp>

#define GLOG_minloglevel 3

// OpenARK Libraries
#include "Version.h"
#ifdef PMDSDK_ENABLED
#include "PMDCamera.h"
#endif
#ifdef RSSDK_ENABLED
#include "SR300Camera.h"
#endif
#ifdef RSSDK2_ENABLED
#include "RS2Camera.h"
#include "MockCamera.h"
#endif

#include "Core.h"
#include "Visualizer.h"
#include "Avatar.h"
#include "HumanDetector.h"

using namespace ark;

static void filterByDepth(cv::Mat& xyz_map, double min_depth, double max_depth) {
    for (int r = 0; r < xyz_map.rows; ++r)
    {
        cv::Vec3f * ptr = xyz_map.ptr<cv::Vec3f>(r);
        for (int c = 0; c < xyz_map.cols; ++c)
        {
            if (ptr[c][2] > max_depth || ptr[c][2] < min_depth) {
                ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
            }
        }
    }
}

static int filterByHeight(cv::Mat& xyz_map, int feet) {
	int skipped = 0;
	for (int r = 0; r < xyz_map.rows; ++r)
	{
		cv::Vec3f * ptr = xyz_map.ptr<cv::Vec3f>(r);
		for (int c = 0; c < xyz_map.cols; ++c)
		{
			if (r < feet) {
				ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
				skipped++;
			}
		}
	}
	return skipped;
}

static void segmentAvatar(const cv::Mat & xyz_map, const std::vector<cv::Point2i> & points_on_target,
    cv::Mat & out) {

	// Filter Background
    cv::Mat floodFillMap = xyz_map.clone();
    filterByDepth(floodFillMap, 2, 4);
	cv::Mat ground = floodFillMap.clone();
	filterByHeight(ground, points_on_target[12].y);
    
	// Remove Plane
	auto viewer = Visualizer::getPCLVisualizer();
	auto groundCloud = util::toPointCloud<pcl::PointXYZ>(ground);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.06);

	seg.setInputCloud(groundCloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return;
	}

	double a = coefficients->values[0];
	double b = coefficients->values[1];
	double c = coefficients->values[2];
	double d = coefficients->values[3];

	for (int r = 0; r < floodFillMap.rows; ++r)
	{
		cv::Vec3f * ptr = floodFillMap.ptr<cv::Vec3f>(r);
		for (int cc = 0; cc < floodFillMap.cols; ++cc)
		{
			double ax = a * ptr[cc][0];
			double bx = b * ptr[cc][1];
			double cx = c * ptr[cc][2];
			double result = ax + bx + cx + d;
			//cout << result << endl;
			if (abs(result) < 0.06) {
				ptr[cc][0] = ptr[cc][1] = ptr[cc][2] = 0.0f;
			}
		}
	}

	// Floodfill Avatar

    std::vector<Point2i> allIndices;
    allIndices.reserve(xyz_map.cols * xyz_map.rows);

    out = cv::Mat(xyz_map.rows, xyz_map.cols, CV_32FC3);

    cv::Mat color(xyz_map.size(), CV_8U);
    color = cv::Scalar(255);

    for (const auto & point : points_on_target) {
        if (point.x < 0 || color.at<unsigned char>(point) != 255) continue;
        util::floodFill(floodFillMap, point, 0.06f,
            &allIndices, nullptr, &out,
            1, 0, 0.0f, &color);
    }
}

template<class T>
static boost::shared_ptr<pcl::PointCloud<T>> denoisePointCloud(boost::shared_ptr<pcl::PointCloud<T>> & human_cloud,
                                     float resolution = 0.05f, int mean_k = 50, double std_dev_mul = 1.0) {
    auto humanCloud_filtered = boost::make_shared<pcl::PointCloud<T>>();
    auto humanCloud_down = boost::make_shared<pcl::PointCloud<T>>();

    pcl::StatisticalOutlierRemoval<T> sor;
    sor.setInputCloud(human_cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev_mul);
    sor.filter(*humanCloud_filtered);

    pcl::VoxelGrid<T> voxel_processor;
    voxel_processor.setInputCloud(humanCloud_filtered);
    voxel_processor.setLeafSize(resolution, resolution, resolution);
    voxel_processor.filter(*humanCloud_down);
    return humanCloud_down;
}

/** OpenPose MPI model output joint indices  */
enum OpenPoseMPIJoint {
    HEAD, NECK, RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST, LEFT_SHOULDER,
    LEFT_ELBOW, LEFT_WRIST, RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE,
    LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, CHEST, BACKGROUND, _COUNT
};

static void toSMPLJoints(const cv::Mat & xyzMap, const std::vector<cv::Point> & mpi_joints,
                         HumanAvatar::EigenCloud_T & out) {
    typedef HumanAvatar::EigenCloud_T cloud;
    typedef HumanAvatar::JointType smpl_j;
    typedef OpenPoseMPIJoint mpi_j;
    
    cloud mpi(mpi_joints.size(), 3);

    for (int i = 0; i < mpi.rows(); ++i) {
        auto r = mpi.row(i);
        if (mpi_joints[i].x == -1) {
            r[0] = -1e12;
        } else {
            Vec3f joint_3d = util::averageAroundPoint(xyzMap, mpi_joints[i], 12);
            r[0] = joint_3d[0];
            r[1] = -joint_3d[1];
            r[2] = -joint_3d[2];
        }
    }

    // 'forward'-facing direction for avatar
    Eigen::Matrix<double, 1, 3> up = mpi.row(mpi_j::NECK) - mpi.row(mpi_j::CHEST);
    up.normalize();
    auto forward = up.cross(mpi.row(mpi_j::RIGHT_HIP) - mpi.row(mpi_j::LEFT_HIP));
    forward.normalize();
 
    double unit = (mpi.row(mpi_j::NECK) - mpi.row(mpi_j::CHEST)).norm() * 0.4;

    out = cloud((int)smpl_j::_COUNT, 3);
    out.row(smpl_j::PELVIS) = mpi.row(mpi_j::LEFT_HIP) * 0.5 + mpi.row(mpi_j::RIGHT_HIP) * 0.5 - forward * unit * 0.42;
    out.row(smpl_j::L_HIP) = mpi.row(mpi_j::LEFT_HIP) * 0.8 + mpi.row(mpi_j::LEFT_KNEE) * 0.2 - forward * unit * 0.3;
    out.row(smpl_j::R_HIP) = mpi.row(mpi_j::RIGHT_HIP) * 0.8 + mpi.row(mpi_j::RIGHT_KNEE) * 0.2 - forward * unit * 0.3;
    out.row(smpl_j::L_KNEE) = mpi.row(mpi_j::LEFT_KNEE);
    out.row(smpl_j::R_KNEE) = mpi.row(mpi_j::RIGHT_KNEE);
    out.row(smpl_j::L_ANKLE) = mpi.row(mpi_j::LEFT_ANKLE);
    out.row(smpl_j::R_ANKLE) = mpi.row(mpi_j::RIGHT_ANKLE);
    out.row(smpl_j::SPINE1) = mpi.row(mpi_j::CHEST) * 0.4 + mpi.row(mpi_j::LEFT_HIP) * 0.3 + mpi.row(mpi_j::RIGHT_HIP) * 0.3 - forward * unit * 0.6;
    out.row(smpl_j::SPINE2) = mpi.row(mpi_j::CHEST) - forward * unit * 0.65;
    out.row(smpl_j::SPINE3) = mpi.row(mpi_j::CHEST) * 0.8 + mpi.row(mpi_j::LEFT_SHOULDER) * 0.1 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.1 - forward * unit * 0.35;
    out.row(smpl_j::HEAD) = mpi.row(mpi_j::NECK) * 0.8 + mpi.row(mpi_j::HEAD) * 0.2 - up * unit * 0.2;
    out.row(smpl_j::NECK) = mpi.row(mpi_j::NECK) * 0.3 + mpi.row(mpi_j::LEFT_SHOULDER) * 0.35 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.35;
    out.row(smpl_j::L_SHOULDER) = mpi.row(mpi_j::LEFT_SHOULDER) - up * unit * 0.25 + forward * unit * 0.1;
    out.row(smpl_j::R_SHOULDER) = mpi.row(mpi_j::RIGHT_SHOULDER) - up * unit * 0.25 + forward * unit * 0.1;
    out.row(smpl_j::L_ELBOW) = mpi.row(mpi_j::LEFT_ELBOW);
    out.row(smpl_j::R_ELBOW) = mpi.row(mpi_j::RIGHT_ELBOW);
    out.row(smpl_j::L_WRIST) = mpi.row(mpi_j::LEFT_WRIST);
    out.row(smpl_j::R_WRIST) = mpi.row(mpi_j::RIGHT_WRIST);
    out.row(smpl_j::L_HAND) = mpi.row(mpi_j::LEFT_WRIST) * 1.4 - mpi.row(mpi_j::LEFT_ELBOW) * 0.4;
    out.row(smpl_j::R_HAND) = mpi.row(mpi_j::RIGHT_WRIST) * 1.4 - mpi.row(mpi_j::RIGHT_ELBOW) * 0.4;
    out.row(smpl_j::L_COLLAR) = mpi.row(mpi_j::LEFT_SHOULDER) * 0.75 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.25 - up * unit * 0.5;
    out.row(smpl_j::R_COLLAR) = mpi.row(mpi_j::LEFT_SHOULDER) * 0.25 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.75 - up * unit * 0.5;
    out.row(smpl_j::L_FOOT) = mpi.row(mpi_j::LEFT_ANKLE) * 1.1 -  mpi.row(mpi_j::LEFT_KNEE) * 0.1 + forward * unit;
    out.row(smpl_j::R_FOOT) = mpi.row(mpi_j::RIGHT_ANKLE) * 1.1 -  mpi.row(mpi_j::RIGHT_KNEE) * 0.1 + forward * unit;

    for (int i = 0; i < out.rows(); ++i) {
        out.row(i) -= forward * unit * 0.2;
        if (out.row(i).x() < -1e10 || std::isnan(out.row(i).x())) {
            out.row(i).x() = NAN;
        }
    }
}

// open a gui for interacting with avatar
void __avatarGUI(const std::string & human_model_path, const std::vector<std::string> & shape_keys)
{
    // build file names and paths
    HumanAvatar ava(human_model_path, shape_keys);

    cv::namedWindow("Body Shape");
    cv::namedWindow("Body Pose");
    std::vector<int> pcw(shape_keys.size(), 1000), p_pcw(shape_keys.size(), 0);

    // define some axes
    const Eigen::Vector3d AXISX(1, 0, 0), AXISY(0, 1, 0), AXISZ(0, 0, 1);

    // Body pose control definitions (currently this control system only supports rotation along one axis per body part)
    const std::vector<std::string> CTRL_NAMES       = {"L HIP",      "R HIP",      "L KNEE",      "R KNEE",      "L ANKLE",      "R ANKLE",      "L ARM",        "R ARM",        "L ELBOW",      "R ELBOW",      "HEAD",      "SPINE2",     "ROOT"};
    using jnt_t = HumanAvatar::JointType;
    const std::vector<jnt_t> CTRL_JNT               = {jnt_t::L_HIP, jnt_t::R_HIP, jnt_t::L_KNEE, jnt_t::R_KNEE, jnt_t::L_ANKLE, jnt_t::R_ANKLE, jnt_t::L_ELBOW, jnt_t::R_ELBOW, jnt_t::L_WRIST, jnt_t::R_WRIST, jnt_t::HEAD, jnt_t::SPINE2, jnt_t::ROOT};
    const std::vector<Eigen::Vector3d> CTRL_AXIS    = {AXISX,        AXISX,        AXISX,         AXISX,         AXISX,          AXISX,          AXISY,          AXISY,          AXISY,          AXISY,          AXISX,       AXISX,         AXISY};
    const int N_CTRL = (int)CTRL_NAMES.size();

    std::vector<int> ctrlw(N_CTRL, 1000), p_ctrlw(N_CTRL, 0);

    // Body shapekeys are defined in SMPL model files.
    int pifx = 0, pify = 0, picx = 0, picy = 0, pframeID = -1;
    cv::resizeWindow("Body Shape", cv::Size(400, 700));
    cv::resizeWindow("Body Pose", cv::Size(400, 700));
    cv::resizeWindow("Body Scale", cv::Size(400, 700));
    for (int i = 0; i < N_CTRL; ++i) {
        cv::createTrackbar(CTRL_NAMES[i], "Body Pose", &ctrlw[i], 2000);
    }
    for (int i = 0; i < (int)pcw.size(); ++i) {
        cv::createTrackbar("PC" + std::to_string(i), "Body Shape", &pcw[i], 2000);
    }

    auto viewer = Visualizer::getPCLVisualizer();

    int vp1 = 0;
    viewer->setWindowName("3D View");
    viewer->setCameraClipDistances(0.0, 1000.0);

    volatile bool interrupt = false;
    viewer->registerKeyboardCallback([&interrupt](const pcl::visualization::KeyboardEvent & evt) {
        unsigned char k = evt.getKeyCode();
        if (k == 'Q' || k == 'q' || k == 27) {
            interrupt = true;
        }
    });

    while (!interrupt) {
        bool controlsChanged = false;
        for (int i = 0; i < N_CTRL; ++i) {
            if (ctrlw[i] != p_ctrlw[i]) {
                controlsChanged = true;
                break;
            }
        }
        for (int i = 0; i < (int)pcw.size(); ++i) {
            if (pcw[i] != p_pcw[i]) {
                controlsChanged = true;
                break;
            }
        }
        if (controlsChanged) {
            viewer->removeAllPointClouds(vp1);
            viewer->removeAllShapes(vp1);
            HumanAvatar::Cloud_T::Ptr depthPC, depthPCPartial;
            ava.update();

            viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), "vp1_cloudHM", vp1);
            ava.visualize(viewer, "vp1_", vp1);

            for (int i = 0; i < N_CTRL; ++i) {
                double angle = (ctrlw[i] - 1000) / 1000.0 * PI;
                ava.setRotation(CTRL_JNT[i], Eigen::AngleAxisd(angle, CTRL_AXIS[i]));
            }

            for (int i = 0; i < (int)pcw.size(); ++i) {
                ava.setKeyWeight(i, (float)(pcw[i] - 1000) / 500.0);
            }

            ava.setCenterPosition(Eigen::Vector3d(0, 0, -3));
            ava.update();

            for (int k = 0; k < (int) pcw.size(); ++k) {
                p_pcw[k] = pcw[k] = (int) (ava.getKeyWeight(k) * 500.0 + 1000);
                cv::setTrackbarPos("PC" + std::to_string(k), "Body Shape", pcw[k]);
            }

            double prior = ava.posePrior.residual(ava.smplParams()).squaredNorm();
            // show pose prior value
            if (!viewer->updateText("-log likelihood: " + std::to_string(prior), 10, 20, 15, 1.0, 1.0, 1.0, "poseprior_disp")) {
                viewer->addText("-log likelihood: " + std::to_string(prior), 10, 20, 15, 1.0, 1.0, 1.0, "poseprior_disp");
            }

            viewer->removePointCloud("vp1_cloudHM");
            viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), "vp1_cloudHM");
            ava.visualize(viewer, "vp1_", vp1);
            viewer->spinOnce();
        }
        for (int i = 0; i < N_CTRL; ++i) p_ctrlw[i] = ctrlw[i];
        for (int i = 0; i < (int)pcw.size(); ++i) p_pcw[i] = pcw[i];

        int k = cv::waitKey(100);
        if (k == 'q' || k == 27) break;
    }
}

int main(int argc, char ** argv) {
    google::InitGoogleLogging(argv[0]);

    printf("Welcome to OpenARK v %s Demo\n\n", VERSION);
    // seed the rng
    srand(time(NULL));

    // gender-neutral model
    const std::string HUMAN_MODEL_PATH = "C:/dev/SMPL/models/basicModel_neutral_lbs_10_207_0_v1.0.0/";

    const std::vector<std::string> SHAPE_KEYS = {"shape000.pcd", "shape001.pcd", "shape002.pcd", 
												 "shape003.pcd", "shape004.pcd", "shape005.pcd",
												 "shape006.pcd", "shape007.pcd", "shape008.pcd", 
												 "shape009.pcd"};

    // UNCOMMENT following line to see the GUI for manipulating SMPL avatar pose, shape, etc.
    //__avatarGUI(HUMAN_MODEL_PATH, SHAPE_KEYS); return 0;

	auto path = "C:\\dev\\OpenARK_dataset\\human-wave-1";
	const auto camera = std::make_shared<MockCamera>(path);

	HumanAvatar ava(HUMAN_MODEL_PATH, SHAPE_KEYS, 2);
	auto viewer = Visualizer::getPCLVisualizer();
	auto vp0 = Visualizer::createPCLViewport(0, 0, 0.7, 1), vp1 = Visualizer::createPCLViewport(0.7, 0, 1, 1);
	int i = 0;
	while (camera->hasNext()) {
		camera->update();
		cv::Mat xyzMap = camera->getXYZMap();
		cv::Mat rgbMap = camera->getRGBMap();
		std::vector<cv::Point> rgbJoints = camera->getJoints();
			
		// segmentation using agglomerate clustering
		cv::Mat out;
		segmentAvatar(xyzMap, rgbJoints, out);

		// convert to PCL point cloud
		auto humanCloudRaw = util::toPointCloud<pcl::PointXYZ>(out, true, true);
		auto humanCloud = denoisePointCloud(humanCloudRaw); // denoise and downsample

		HumanAvatar::EigenCloud_T xyzJoints;
		toSMPLJoints(out, rgbJoints, xyzJoints);

		// show images
		for (size_t n = 0; n < rgbJoints.size(); ++n) {
			cv::circle(out, rgbJoints[n], 8, cv::Scalar(0, 255, 255), -1);
			cv::putText(out, std::to_string(n), rgbJoints[n], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 0));
		}

		cv::imshow("Fill", out);
		cv::Mat xyzVis;
		Visualizer::visualizeXYZMap(xyzMap, xyzVis, 7.5f);
		cv::imshow("Depth Map", xyzVis);
		cv::imshow("RGB", rgbMap);

        if (i == 0) {
            ava.setCenterPosition(util::cloudCenter(humanCloudRaw));
            ava.update();
            ava.alignToJoints(xyzJoints);
        }
        else {
            ava.updateJointsPrior(xyzJoints);
        }
		ava.update();


		// visualize
		if (i == 0) {
			std::cout << "Fitting" << std::endl;
			ava.fit(humanCloud);
		}
		else {
			std::cout << "Tracking" << std::endl;
			ava.fitTrack(humanCloud);
		}
			
		ava.visualize(viewer, "o1_ava_", vp1);
		ava.visualize(viewer, "ava_", vp0);

		const std::string DATA_CLOUD_NAME = "data_cloud";
        viewer->removePointCloud(DATA_CLOUD_NAME, vp0);
        viewer->addPointCloud<pcl::PointXYZ>(humanCloud, DATA_CLOUD_NAME, vp0);

		viewer->spinOnce();
		int c = cv::waitKey(1);
		i++;

		if (c == 'Q' || c == 27) {
			/*** Loop Break Condition ***/
			break;
		}
	}

    cv::destroyAllWindows();
    return 0;
}
