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

	//// turn on the camera
	//camera->beginCapture();

	//// main demo loop
	//std::vector<cv::Mat> xyzMaps;
	//std::vector<cv::Mat> rgbMaps;
	//while (true)
	//{
	//	++currFrame;

	//	// get latest image from the camera
	//	cv::Mat xyzMap = camera->getXYZMap();
	//	cv::Mat rgbMap = camera->getRGBMap();

	//	std::stringstream ss;
	//	ss << "C:\\dev\\OpenARK_dataset\\human-basic-rgb-D435\\capture_" << currFrame << ".yml";
	//	std::string curr_file_name = ss.str();
	//	std::cout << curr_file_name << std::endl;

	//	cv::FileStorage fs(curr_file_name, cv::FileStorage::WRITE);
	//	fs << "xyz_map" << xyzMap;
	//	fs << "rgb_map" << rgbMap;
	//	fs.release();

	//	// show visualizations
	//	if (!xyzMap.empty()) {
	//		cv::imshow(camera->getModelName() + " Depth Map", xyzMap);
	//	} 

	//	if (!rgbMap.empty()) {
	//		cv::imshow(camera->getModelName() + " RGB Map", rgbMap);
	//	}
	//	/**** End: Visualization ****/

	//	/**** Start: Controls ****/
	//	int c = cv::waitKey(1);

	//	// make case insensitive
	//	if (c >= 'a' && c <= 'z') c &= 0xdf;

	//	// 27 is ESC
	//	if (c == 'Q' || c == 27) {
	//		/*** Loop Break Condition ***/
	//		break;
	//	}
	//	/**** End: Controls ****/
	//}

	//camera->endCapture();

	//ASSERT(xyzMaps.size() == rgbMaps.size(), "Depth map and RGB map are not in sync!");

	//for (int i = 0; i < xyzMaps.size(); ++i) {
	//	std::stringstream ss;
	//	ss << "C:\\dev\\OpenARK_dataset\\human-basic-rgb-D435\\capture_" << std::setw(2) << std::setfill('0') << i << ".yml";
	//	std::string curr_file_name = ss.str();
	//	std::cout << curr_file_name << std::endl;

	//	cv::FileStorage fs(curr_file_name, cv::FileStorage::WRITE);
	//	fs << "xyz_map" << xyzMaps[i];
	//	fs << "rgb_map" << rgbMaps[i];
	//	fs.release();
	//}

	std::vector<std::string> file_names;
	boost::filesystem::path image_dir("C:\\dev\\OpenARK_dataset\\human-basic-rgb-D435\\");

	if (is_directory(image_dir)) {
		
		boost::filesystem::directory_iterator end_iter;
		for (boost::filesystem::directory_iterator dir_itr(image_dir); dir_itr != end_iter; ++dir_itr) {
			const auto& next_path = dir_itr->path().generic_string();
			file_names.emplace_back(next_path);
		}
		std::sort(file_names.begin(), file_names.end());
	}

	std::shared_ptr<HumanDetector> human_detector = std::make_shared<HumanDetector>();
	for (const auto& filename : file_names) {
		std::cout << filename << std::endl;
		cv::Mat rgb_map, xyz_map;
		cv::FileStorage fs2(filename, cv::FileStorage::READ);
		fs2["rgb_map"] >> rgb_map;
		fs2.release();
		
		
		human_detector->update(rgb_map);
		std::vector<cv::Point> rgbJoints = human_detector->getHumanBodies()[0]->MPIISkeleton2D;
		for (const auto& joint : rgbJoints) {
			cv::circle(rgb_map, joint, 2, cv::Scalar(255, 0, 0), 2);
		}
		cv::imshow("RGB Frame Data", rgb_map);
		
		cv::FileStorage fs3(filename, cv::FileStorage::APPEND);
		fs3 << "joints" << rgbJoints;
		fs3.release();
		cv::waitKey(1);

		rgbJoints.clear();
	}
	int c = cv::waitKey(1);
	cv::destroyAllWindows();
	return 0;
}
