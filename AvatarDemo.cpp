#include "stdafx.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

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

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;


void alignImages(cv::Mat &im1, cv::Mat &im2, cv::Mat &im1Reg, cv::Mat &h)
{

	// Convert images to grayscale
	cv::Mat im1Gray, im2Gray;
	cv::cvtColor(im1, im1Gray, CV_BGR2GRAY);
	cv::cvtColor(im2, im2Gray, CV_BGR2GRAY);

	// Variables to store keypoints and descriptors
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;

	// Detect ORB features and compute descriptors.
	cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
	orb->detectAndCompute(im1Gray, cv::Mat(), keypoints1, descriptors1);
	orb->detectAndCompute(im2Gray, cv::Mat(), keypoints2, descriptors2);

	// Match features.
	std::vector<cv::DMatch> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	matcher->match(descriptors1, descriptors2, matches, cv::Mat());

	// Sort matches by score
	std::sort(matches.begin(), matches.end());

	// Remove not so good matches
	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
	matches.erase(matches.begin() + numGoodMatches, matches.end());


	// Draw top matches
	cv::Mat imMatches;
	cv::drawMatches(im1, keypoints1, im2, keypoints2, matches, imMatches);
	cv::imshow("matches ", imMatches);

	// Extract location of good matches
	std::vector<cv::Point2f> points1, points2;

	for (size_t i = 0; i < matches.size(); i++)
	{
		points1.push_back(keypoints1[matches[i].queryIdx].pt);
		points2.push_back(keypoints2[matches[i].trainIdx].pt);
	}

	// Create partial point clounds based on matches

	// Get correspondences between points
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
	TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);

	// Get initialization on the full point clouds
	pcl::transformPointCloud(*src, *icp_cloud, initial_pose);

	// Perform ICP to get the final pose
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	//// Find homography
	//h = cv::findHomography(points1, points2, cv::RANSAC);

	//// Use homography to warp image
	//cv::warpPerspective(im1, im1Reg, h, im2.size());
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

	auto path = "C:\\dev\\OpenARK_dataset\\human-dance-random";
	const auto camera = std::make_shared<RS2Camera>(true);
	//std::shared_ptr<HumanDetector> human_detector = std::make_shared<HumanDetector>();

	//HumanAvatar ava(HUMAN_MODEL_PATH, SHAPE_KEYS, 2);
	//auto viewer = Visualizer::getPCLVisualizer();
	//auto vp0 = Visualizer::createPCLViewport(0, 0, 0.7, 1), vp1 = Visualizer::createPCLViewport(0.7, 0, 1, 1);
	int i = 0;
	camera->beginCapture();
	while (true)
	{
		/*cv::Mat xyzMap = camera->getXYZMap();
		cv::Mat rgbMap = camera->getRGBMap();*/

		std::vector<cv::Mat> xyzMaps(camera->getXYZMaps());
		std::vector<cv::Mat> rgbMaps(camera->getRGBMaps());

		// show visualizations
		if (xyzMaps.size() == 2 && !xyzMaps[0].empty() && !xyzMaps[1].empty()) {
			cv::imshow(camera->getModelName() + " Depth Map 1", xyzMaps[0]);
			cv::imshow(camera->getModelName() + " Depth Map 2", xyzMaps[1]);
		}

		if (rgbMaps.size() == 2 && !rgbMaps[0].empty() && !rgbMaps[1].empty()) {
			cv::imshow(camera->getModelName() + " RGB Map 1", rgbMaps[0]);
			cv::imshow(camera->getModelName() + " RGB Map 2", rgbMaps[1]);

			cv::Mat imReg, h;
			// Align images
			//cout << "Aligning images ..." << endl;
			alignImages(rgbMaps[0], rgbMaps[1], imReg, h);
			//cout << "Estimated homography : \n" << h << endl;
		}

		// Print estimated homography
		//cout << "Estimated homography : \n" << h << endl;

		/**** End: Visualization ****/

		/**** Start: Controls ****/
		int c = cv::waitKey(1);

		// make case insensitive
		if (c >= 'a' && c <= 'z') c &= 0xdf;

		// 27 is ESC
		if (c == 'Q' || c == 27) {
			/*** Loop Break Condition ***/
			break;
		}
		/**** End: Controls ****/
	}
	camera->endCapture();
	//while (camera->hasNext()) {
	//	camera->update();
	//	cv::Mat xyzMap = camera->getXYZMap();
	//	cv::Mat rgbMap = camera->getRGBMap();
	//	std::vector<cv::Point> rgbJoints = camera->getJoints();

	//	// Tracking code
	//	human_detector->update(xyzMap, rgbMap, rgbJoints);
	//	std::shared_ptr<HumanAvatar> avatar_model = human_detector->getAvatarModel();
	//	// render the human in GUI
	//	avatar_model->visualize(viewer, "o1_ava_", vp1);
	//	avatar_model->visualize(viewer, "ava_", vp0);
	//	viewer->spinOnce();

	//	int c = cv::waitKey(1);
	//	i++;

	//	if (c == 'Q' || c == 27) {
	//		/*** Loop Break Condition ***/
	//		break;
	//	}
	//}

    cv::destroyAllWindows();
    return 0;
}
