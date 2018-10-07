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
	const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_neutral_lbs_10_207_0_v1.0.0/";
	// male model
	//const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_m_lbs_10_207_0_v1.0.0/";
	// female model
	//const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_f_lbs_10_207_0_v1.0.0/";

	const std::vector<std::string> SHAPE_KEYS = { "shape000.pcd", "shape001.pcd",
		"shape002.pcd", "shape003.pcd", "shape004.pcd",
		"shape005.pcd", "shape006.pcd", "shape007.pcd",
		"shape008.pcd", "shape009.pcd"
	};

	// initialize parameters
	DetectionParams::Ptr params = DetectionParams::create();

	PlaneDetector::Ptr planeDetector = std::make_shared<PlaneDetector>();
	cv::Mat xyzMap, rgbMap;

	std::string imgPath = argc > 1 ? argv[1] : IMG_PATH;
	cv::FileStorage fs2(imgPath, cv::FileStorage::READ);
	fs2["xyz_map"] >> xyzMap;
	fs2["rgb_map"] >> rgbMap;
	fs2.release();
	

	cv::Mat floodFillMap = xyzMap.clone();
	filter_by_depth(floodFillMap, 1, 3);
	cv::imshow("Mid", floodFillMap);

	planeDetector->update(xyzMap);
	const std::vector<FramePlane::Ptr> & planes = planeDetector->getPlanes();
	//Visualizer::visualizeNormalMap(planeDetector->getNormalMap(), xyzMap, params->normalResolution);
	/*auto viewer = Visualizer::getPCLVisualizer();
	auto humanCloud = util::toPointCloud<pcl::PointXYZ>(xyzMap, true, true);
	viewer->addPointCloud<pcl::PointXYZ>(humanCloud, "XYZ Map");
	viewer->spinOnce();*/
	
	if (planes.size()) {
		for (FramePlane::Ptr plane : planes) {
			util::removePlane<Vec3f>(floodFillMap, floodFillMap, plane->equation, 0.0015);
		}
	}

	std::vector<Point2i> allIndices;
	allIndices.reserve(xyzMap.cols * xyzMap.rows);

	cv::Mat out(xyzMap.rows, xyzMap.cols, CV_32FC3);
	//cv::Mat out = floodFillMap.clone();
	int numPts = util::floodFill(floodFillMap, Point2i(410, 200), 2.0f,
		&allIndices, nullptr, &out,
		1, 0, 200.0f);

	cv::circle(rgbMap, Point(410, 200), 2, cv::Scalar(1, 1, 1), 2);
	//cv::imshow("Mid", floodFillMap);
	cv::imshow("RGB Map", rgbMap);
	cv::imshow("Fill", out);
	cv::imshow("Depth Map", xyzMap);

	/**
	auto humanCloud = util::toPointCloud<pcl::PointXYZ>(out, true, true);
	auto humanCloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	auto humanCloud_down = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	auto viewer = Visualizer::getPCLVisualizer();

	HumanAvatar ava(HUMAN_MODEL_PATH, SHAPE_KEYS);

	ava.setCenterPosition(util::cloudCenter(humanCloud));
	ava.update();

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(humanCloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*humanCloud_filtered);

	pcl::VoxelGrid<pcl::PointXYZ> voxel_processor;
	voxel_processor.setInputCloud(humanCloud_filtered);
	voxel_processor.setLeafSize(0.03f, 0.03f, 0.03f);
	voxel_processor.filter(*humanCloud_down);

	const std::string MODEL_CLOUD_NAME = "model_cloud", DATA_CLOUD_NAME = "data_cloud";

	viewer->addPointCloud<pcl::PointXYZ>(humanCloud_down, DATA_CLOUD_NAME);
	std::cout << "Data (Human) Points: " << humanCloud_down->size() << ". Model (Avatar) Points: " << ava.getCloud()->size() << "\n";
	viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME);
	ava.fit(humanCloud_down);
	ava.visualize();
	viewer->removePointCloud(MODEL_CLOUD_NAME);
	viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME);
	viewer->spinOnce();
	*/
	cv::waitKey(0);

	cv::destroyAllWindows();
	return 0;
}