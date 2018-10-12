#include "stdafx.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <opencv2/ximgproc.hpp>

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

using namespace ark;

static void filter_by_depth(cv::Mat& xyz_map, double min_depth, double max_depth) {
    for (int r = 0; r < xyz_map.rows; ++r)
    {
        Vec3f * ptr = xyz_map.ptr<Vec3f>(r);
        for (int c = 0; c < xyz_map.cols; ++c)
        {
            if (ptr[c][2] > max_depth || ptr[c][2] < min_depth || r > 410) {
                ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
            }
        }
    }
}

static void rgb_pose_estimation(cv::Mat frame) {
    static const int POSE_PAIRS[14][2] =
    {
        { 0,1 },{ 1,2 },{ 2,3 },

        { 3,4 },{ 1,5 },{ 5,6 },

        { 6,7 },{ 1,14 },{ 14,8 },{ 8,9 },

        { 9,10 },{ 14,11 },{ 11,12 },{ 12,13 }
    };

    int nPoints = 15;

    static const std::string protoFile = "C:\\dev\\openpose\\models\\pose\\mpi\\pose_deploy_linevec_faster_4_stages.prototxt";
    static const std::string weightsFile = "C:\\dev\\openpose\\models\\pose\\mpi\\pose_iter_160000.caffemodel";

    cv::dnn::Net net = cv::dnn::readNetFromCaffe(protoFile, weightsFile);


    //cv::Mat frame = cv::imread("C:\\dev\\OpenARK_dataset\\human3.jpg");
    cv::Mat frameCopy = frame.clone();
    int frameWidth = frame.cols;
    int frameHeight = frame.rows;

    // Specify the input image dimensions
    int inWidth = frame.cols;
    int inHeight = frame.rows;
    float thresh = 0.3;
    //cv::ximgproc::dtFilter(frame, frame, frame, 1.0, 1.0);

    cv::imshow("Human", frame);

    // Prepare the frame to be fed to the network
    cv::Mat inpBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(inWidth, inHeight), cv::Scalar(0, 0, 0));

    // Set the prepared object as the input blob of the network
    net.setInput(inpBlob);

    cv::Mat output = net.forward();
    int H = output.size[2];
    int W = output.size[3];

    // find the position of the body parts
    std::vector<cv::Point> points(nPoints);
    for (int n = 0; n < nPoints; n++) {
        // Probability map of corresponding body's part.

        cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));

        cv::Point2f p(-1, -1);
        cv::Point maxLoc;
        double prob;

        minMaxLoc(probMap, 0, &prob, 0, &maxLoc);

        if (prob > thresh) {
            p = maxLoc;
            p.x *= (float)frameWidth / W;
            p.y *= (float)frameHeight / H;

            cv::circle(frameCopy, cv::Point((int)p.x, (int)p.y), 8, cv::Scalar(0, 255, 255), -1);
            cv::putText(frameCopy, cv::format("%d", n), cv::Point((int)p.x, (int)p.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
        points[n] = p;
    }

    int nPairs = sizeof(POSE_PAIRS) / sizeof(POSE_PAIRS[0]);

    for (int n = 0; n < nPairs; n++) {

        // lookup 2 connected body/hand parts

        cv::Point2f partA = points[POSE_PAIRS[n][0]];
        cv::Point2f partB = points[POSE_PAIRS[n][1]];

        if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0)
            continue;


        cv::line(frame, partA, partB, cv::Scalar(0, 255, 255), 8);
        cv::circle(frame, partA, 8, cv::Scalar(0, 0, 255), -1);
        cv::circle(frame, partB, 8, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("Output-Keypoints", frameCopy);
    cv::imshow("Output-Skeleton", frame);
}

static void face_direction()
{
	cv::CascadeClassifier faceDetector("C:\\opencv\\data\\haarcascades\\haarcascade_frontalface_alt2.xml");

	// Create an instance of Facemark
	cv::Ptr<cv::face::Facemark> facemark = cv::face::FacemarkLBF::create();

	// Load landmark detector
	facemark->loadModel("C:\\dev\\OpenARK_dataset\\lbfmodel.yaml");

	cv::VideoCapture cam(0);
	cv::Mat frame, gray;

	while (cam.read(frame)) {
		// Find face
		std::vector<cv::Rect> faces;

		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		// Detect faces
		faceDetector.detectMultiScale(gray, faces);

		// There can be more than one face in the image. Hence, we 
		// use a vector of vector of points. 
		std::vector<std::vector<cv::Point2f>> landmarks;

		// Run landmark detector
		bool success = facemark->fit(frame, faces, landmarks);

		std::vector<cv::Point2d> image_points;
		if (success && landmarks[0].size() == 68) {
			Visualizer::visualizeFaceLandmarks(frame, landmarks[0]);

			image_points.push_back(landmarks[0][30]);    // Nose tip
			image_points.push_back(landmarks[0][8]);     // Chin
			image_points.push_back(landmarks[0][36]);    // Left eye left corner
			image_points.push_back(landmarks[0][45]);    // Right eye right corner
			image_points.push_back(landmarks[0][60]);    // Left Mouth corner
			image_points.push_back(landmarks[0][64]);    // Right mouth corner
		}
		else {
			cv::imshow("Facial Landmark Detection", frame);
			continue;
		}

		// 3D model points.

		std::vector<cv::Point3d> model_points;

		model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
		model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
		model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
		model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
		model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
		model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

		// Camera internals
		double focal_length = frame.cols; // Approximate focal length.
		Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
		cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
		cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

		// Output rotation and translation
		cv::Mat rotation_vector; // Rotation in axis-angle form
		cv::Mat translation_vector;

		// Solve for pose
		cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);


		std::vector<cv::Point3d> nose_end_point3D;
		std::vector<cv::Point2d> nose_end_point2D;
		nose_end_point3D.push_back(cv::Point3d(0, 0, 1000.0));

		cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);

		for (int i = 0; i < image_points.size(); i++) {
			circle(frame, image_points[i], 3, cv::Scalar(0, 0, 255), -1);
		}

		cv::line(frame, image_points[0], nose_end_point2D[0], cv::Scalar(255, 0, 0), 2);

		cv::imshow("Facial Landmark Detection", frame);
	}
}

int main(int argc, char ** argv) {
	google::InitGoogleLogging(argv[0]);

	printf("Welcome to OpenARK v %s Demo\n\n", VERSION);
	// seed the rng
	srand(time(NULL));

	const std::string IMG_PATH = "C:\\dev\\OpenARK_dataset\\human-basic-rgb-D435\\capture_14.yml";
	// gender-neutral model
	const std::string HUMAN_MODEL_PATH = "C:/dev/SMPL/models/basicModel_neutral_lbs_10_207_0_v1.0.0/";
	// male model
	//const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_m_lbs_10_207_0_v1.0.0/";
	// female model
	//const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_f_lbs_10_207_0_v1.0.0/";

	const std::vector<std::string> SHAPE_KEYS = { "shape000.pcd", "shape001.pcd",
		"shape002.pcd", "shape003.pcd", "shape004.pcd",
		"shape005.pcd", "shape006.pcd", "shape007.pcd",
		"shape008.pcd", "shape009.pcd"
	};

	// ** create avatar instance **
	HumanAvatar ava(HUMAN_MODEL_PATH, SHAPE_KEYS, 2);

	// ** set up PCL viewer **
	auto viewer = Visualizer::getPCLVisualizer();
	int vp1 = 0;
	viewer->setWindowName("3D View");
	viewer->setCameraClipDistances(0.0, 1000.0);

	volatile bool interrupt = false;
	viewer->registerKeyboardCallback([&interrupt](const pcl::visualization::KeyboardEvent & evt) {
		unsigned char k = evt.getKeyCode();
		if (k == 'Q' || k == 'q' || k == 27) {
			// force quit
			interrupt = true;
		}
	});

	// ** create control windows **
	cv::namedWindow("Body Shape");
	cv::namedWindow("Body Pose");

	std::vector<int> pcw(SHAPE_KEYS.size(), 1000), p_pcw(SHAPE_KEYS.size(), 0);

	// Define axes
	const Eigen::Vector3d AXISX(1, 0, 0), AXISY(0, 1, 0), AXISZ(0, 0, 1);

	// Body pose control definitions (currently this control system only supports rotation along one axis per body part)
	const std::vector<std::string> CTRL_NAMES = { "L LEG",       "R LEG",       "L KNEE",       "R KNEE",       "L ANKLE",     "R ANKLE",     "L ARM",        "R ARM",        "L ELBOW",      "R ELBOW",      "HEAD",      "SPINE2" };
	using jnt_t = HumanAvatar::JointType;
	const std::vector<jnt_t> CTRL_JNT = { jnt_t::R_KNEE, jnt_t::L_KNEE, jnt_t::R_ANKLE, jnt_t::L_ANKLE, jnt_t::R_FOOT, jnt_t::L_FOOT, jnt_t::R_ELBOW, jnt_t::L_ELBOW, jnt_t::R_WRIST, jnt_t::L_WRIST, jnt_t::HEAD, jnt_t::SPINE2 };
	const std::vector<Eigen::Vector3d> CTRL_AXIS = { AXISX,         AXISX,         AXISX,           AXISX,         AXISX,         AXISX,         AXISY,          AXISY,          AXISY,          AXISY,          AXISX,       AXISX };
	const int N_CTRL = (int)CTRL_NAMES.size();
	std::vector<int> ctrlw(N_CTRL, 1000), p_ctrlw(N_CTRL, 0);

	// Body shapekeys are defined in SMPL model files.
	int pifx = 0, pify = 0, picx = 0, picy = 0, pframeID = -1;
	cv::resizeWindow("Body Shape", cv::Size(400, 700));
	cv::resizeWindow("Body Pose", cv::Size(400, 700));
	for (int i = 0; i < N_CTRL; ++i) {
		cv::createTrackbar(CTRL_NAMES[i], "Body Pose", &ctrlw[i], 2000);
	}
	for (int i = 0; i < (int)pcw.size(); ++i) {
		cv::createTrackbar("PC" + std::to_string(i), "Body Shape", &pcw[i], 2000);
	}

	// ** Primary control loop **
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
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();

			HumanAvatar::Cloud_T::Ptr depthPC;

			ava.update();

			viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), "vp1_cloudHM");
			ava.visualize(viewer, "vp1_");

			for (int i = 0; i < N_CTRL; ++i) {
				double angle = (ctrlw[i] - 1000) / 1000.0 * PI;
				ava.setRotation(CTRL_JNT[i], Eigen::AngleAxisd(angle, CTRL_AXIS[i]));
			}

			for (int i = 0; i < (int)pcw.size(); ++i) {
				ava.setKeyWeight(i, (float)(pcw[i] - 1000) / 500.0);
			}

			ava.update();

			viewer->spinOnce(1, true);
		}

		for (int i = 0; i < N_CTRL; ++i) p_ctrlw[i] = ctrlw[i];
		for (int i = 0; i < (int)pcw.size(); ++i) p_pcw[i] = pcw[i];

		int k = cv::waitKey(100);
		if (k == 'q' || k == 27) break;
	}

	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	return 0;
}