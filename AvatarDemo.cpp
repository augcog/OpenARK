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
	camera = std::make_shared<RS2Camera>();
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

	// turn on the camera
	camera->beginCapture();

	// main demo loop
	while (true)
	{
		++currFrame;

		// get latest image from the camera
		cv::Mat xyzMap = camera->getXYZMap();
		/*std::stringstream ss;
		ss << "C:\\dev\\OpenARK_Dataset\\capture_" << currFrame << ".yml";
		std::string curr_file_name = ss.str();
		std::cout << curr_file_name << std::endl;

		cv::FileStorage fs(curr_file_name, cv::FileStorage::WRITE);
		fs << "xyz_map" << xyzMap;
		fs.release();*/

		// show visualizations
		if (!xyzMap.empty()) {
			cv::imshow(camera->getModelName() + " Depth Map", xyzMap);
		}
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
		else if (c >= '0' && c <= '3') {
			backgroundStyle = c - '0';
		}

		switch (c) {
		case 'P':
			showPlanes ^= 1; break;
		case 'H':
			showHands ^= 1; break;
		case 'S':
			useSVM ^= 1; break;
		case 'C':
			useEdgeConn ^= 1; break;
		case 'A':
			showArea ^= 1; break;
		case ' ':
			playing ^= 1;
			// reset frame number
			if (playing) currFrame = -1;
			break;
		}

		/**** End: Controls ****/
	}

	camera->endCapture();

	cv::destroyAllWindows();
	return 0;
}
