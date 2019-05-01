#include "stdafx.h"

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
#endif
#ifdef MOCKCAMERA_ENABLED
#include "MockCamera.h"
#endif

#include "opencv2/imgcodecs.hpp"

#include "Core.h"
#include "Visualizer.h"
#include "StreamingAverager.h"
#include "HumanDetector.h"


using namespace ark;

int main() {
	printf("Welcome to OpenARK v %s Demo\n\n", VERSION);
	printf("CONTROLS:\nQ or ESC to quit, P to show/hide planes, H to show/hide hands, SPACE to play/pause\n\n");
	printf("VIEWER BACKGROUNDS:\n1 = RGB/IR Image, 2 = Depth Image, 3 = Normal Map, 0 = None\n\n");
	printf("HAND DETECTION OPTIONS:\nS = Enable/Disable SVM, C = Enforce/Unenforce Edge Connected Criterion\n\n");
	printf("MISCELLANEOUS:\nA = Measure Surface Area (Of Hands and Planes)\n");

	// seed the rng
	srand(time(NULL));

	// initialize the camera
	DepthCamera::Ptr camera;

#if defined(RSSDK2_ENABLED)
	camera = std::make_shared<RS2Camera>(true);
#elif defined(RSSDK_ENABLED)
	ASSERT(strcmp(OPENARK_CAMERA_TYPE, "sr300") == 0, "Unsupported RealSense camera type.");
	camera = std::make_shared<SR300Camera>();
#elif defined(PMDSDK_ENABLED)
	camera = std::make_shared<PMDCamera>();
#elif defined(MOCKCAMERA_ENABLED)
	std::string path = "C:\\dev\\OpenARK_Dataset\\";
	camera = std::make_shared<MockCamera>(path);
#endif

	// initialize parameters
	DetectionParams::Ptr params = camera->getDefaultParams(); // default parameters for camera

															  // store frame & FPS information
	const int FPS_CYCLE_FRAMES = 8; // number of frames to average FPS over (FPS 'cycle' length)
	using ms = std::chrono::duration<float, std::milli>;
	using time_point = std::chrono::high_resolution_clock::time_point;

	std::chrono::high_resolution_clock timer = std::chrono::high_resolution_clock();
	time_point currCycleStartTime = timer.now(); // start time of current cycle

	float currFPS; // current FPS

	int currFrame = 0; // current frame number (since launch/last pause)
	int backgroundStyle = 1; // background style: 0=none, 1=ir, 2=depth, 3=normal

							 // option flags
	bool showHands = true, showPlanes = false, useSVM = true, useEdgeConn = false, showArea = false, playing = true;

	const std::string directory_path = "C:\\dev\\OpenARK_dataset\\human-walk\\";

	// turn on the camera
	camera->beginCapture();

	// Read in camera input and save it to the buffer
	std::vector<cv::Mat> xyzMaps;
	std::vector<cv::Mat> rgbMaps;
	while (true)
	{
		++currFrame;

		// get latest image from the camera
		cv::Mat xyzMap = camera->getXYZMap();
		cv::Mat rgbMap = camera->getRGBMap();

		std::stringstream ss;
		ss << directory_path << currFrame << ".yml";
		std::string curr_file_name = ss.str();

		xyzMaps.push_back(xyzMap);
		rgbMaps.push_back(rgbMap);

		// show visualizations
		if (!xyzMap.empty()) {
			cv::imshow(camera->getModelName() + " Depth Map", xyzMap);
		} 

		if (!rgbMap.empty()) {
			cv::imshow(camera->getModelName() + " RGB Map", rgbMap);
		}
		/**** End: Visualization ****/

		/**** Start: Controls ****/
		int c = cv::waitKey(66);

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
	
	//// Write the captured frames to disk
	//ASSERT(xyzMaps.size() == rgbMaps.size(), "Depth map and RGB map are not in sync!");

	//std::string depth_path = directory_path + "depth\\";
	//std::string rgb_path = directory_path + "rgb\\";
	//if (boost::filesystem::exists(depth_path)) {
	//	boost::filesystem::create_directories(depth_path);
	//} if (boost::filesystem::exists(rgb_path)) {
	//	boost::filesystem::create_directories(rgb_path);
	//}

	//for (int i = 0; i < xyzMaps.size(); ++i) {
	//	cout << "Writing " << i << " / " << xyzMaps.size() << endl;
	//	std::stringstream ss_depth, ss_rgb;
	//	ss_depth << directory_path << "depth\\" << "depth_" << std::setw(4) << std::setfill('0') << i << ".exr";
	//	ss_rgb << directory_path << "rgb\\" << "rgb_" << std::setw(4) << std::setfill('0') << i << ".jpg";
	//	cout << "Writing " << ss_depth.str() << endl;
	//	cv::imwrite(ss_depth.str(), xyzMaps[i]);
	//	cv::imwrite(ss_rgb.str(), rgbMaps[i]);
	//}

	//// Read in all the rgb images
	//std::vector<std::string> rgb_files;
	//boost::filesystem::path rgb_dir(directory_path + "rgb\\");

	//if (is_directory(rgb_dir)) {
	//	boost::filesystem::directory_iterator end_iter;
	//	for (boost::filesystem::directory_iterator dir_itr(rgb_dir); dir_itr != end_iter; ++dir_itr) {
	//		const auto& next_path = dir_itr->path().generic_string();
	//		rgb_files.emplace_back(next_path);
	//	}
	//	std::sort(rgb_files.begin(), rgb_files.end());
	//}
	//std::vector<std::string> depth_files;
	//boost::filesystem::path depth_dir(directory_path + "depth\\");

	//if (is_directory(depth_dir)) {
	//	boost::filesystem::directory_iterator end_iter;
	//	for (boost::filesystem::directory_iterator dir_itr(depth_dir); dir_itr != end_iter; ++dir_itr) {
	//		const auto& next_path = dir_itr->path().generic_string();
	//		depth_files.emplace_back(next_path);
	//	}
	//	std::sort(depth_files.begin(), depth_files.end());
	//}
	//ASSERT(depth_files.size() == rgb_files.size());

	//// Run neural network to predict where the human joints are
	//std::string joint_path = directory_path + "joint\\";
	//if (boost::filesystem::exists(joint_path)) {
	//	boost::filesystem::create_directories(joint_path);
	//}

	//std::shared_ptr<HumanDetector> human_detector = std::make_shared<HumanDetector>();
	//int frame = 0;
	//for (int i = 0; i < depth_files.size(); i++) {
	//	const auto rgb_filename = rgb_files[i];
	//	const auto depth_filename = depth_files[i];
	//	std::cout << rgb_filename << std::endl;
	//	cv::Mat rgb_map_raw, rgb_map, xyz_map;
	//	rgb_map = cv::imread(rgb_filename);
	//	xyz_map = cv::imread(depth_filename, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

	//	double alpha = 1.5; /*< Simple contrast control */
	//	int beta = -20;       /*< Simple brightness control */
	//	

	//	//rgb_map_raw.convertTo(rgb_map, -1, alpha, beta);

	//	human_detector->getHumanBodies().clear();
	//	//cout << human_detector->getHumanBodies().size() << endl;
	//	human_detector->update(rgb_map);
	//	std::vector<cv::Point> rgbJoints;
	//	if (human_detector->getHumanBodies().size() != 0) {
	//		int front_id = -1, min_dist = 100;
	//		for (int i = 0; i < human_detector->getHumanBodies().size(); i++) {
	//			cv::Point pt(human_detector->getHumanBodies()[i]->MPIISkeleton2D[1].x, human_detector->getHumanBodies()[i]->MPIISkeleton2D[1].y);
	//			//cout << xyz_map.at<cv::Vec3f>(pt)[2] << endl;
	//			if (xyz_map.at<cv::Vec3f>(pt)[2] < min_dist) {
	//				front_id = i;
	//				min_dist = xyz_map.at<cv::Vec3f>(pt)[2];
	//			}
	//		}
	//		if (front_id == -1) {
	//			cout << "No humans found" << endl;
	//			continue;
	//		}
	//		if (min_dist > 8 || min_dist < 1) {
	//			cout << "Not in range" << endl;
	//			continue;
	//		}
	//		rgbJoints = human_detector->getHumanBodies()[front_id]->MPIISkeleton2D;
	//		for (const auto& joint : rgbJoints) {
	//			//cout << joint << endl;
	//			cv::circle(rgb_map, joint, 2, cv::Scalar(255, 0, 0), 2);
	//		}
	//		cv::imshow("RGB Frame Data", rgb_map);
	//		
	//	}
	//	
	//	
	//	std::stringstream ss_joint;
	//	ss_joint << directory_path << "joint\\" << "joint_" << std::setw(4) << std::setfill('0') << frame << ".yml";
	//	cv::FileStorage fs3(ss_joint.str(), cv::FileStorage::WRITE);
	//	fs3 << "joints" << rgbJoints;
	//	fs3.release();
	//	rgbJoints.clear();
	//	
	//	frame++;
	//	cv::waitKey(1);
	//}
	int c = cv::waitKey(1);
	cv::destroyAllWindows();
	return 0;
}
