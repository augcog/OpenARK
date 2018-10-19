#include "HumanDetector.h"

namespace ark {
	HumanDetector::HumanDetector(DetectionParams::Ptr params) {
		// Since we have seen no humans previously, we set this to default value
		lastHumanDetectionBox = cv::Rect(0, 0, 0, 0);

		// Load the human HOG descriptor
		humanHOG.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
	}

	void HumanDetector::detect(cv::Mat & image) {
		// Get the human area via HOG
		//detectHumanHOG(image);

		// Feed the HOG area into the Body Pose Estimation
		//detectBodyPose(image);

		// Feed the HOG area into the Head Pose Estimation
		detectHeadPose(image);
	}

	void HumanDetector::detectHumanHOG(const cv::Mat& frame) {
		cout << "HOG Call" << endl;
		cv::Mat img, original;
		
		// copy the rgb image where we'll applied the rectangles
		img = frame.clone();

		// convert to grayscale
		cvtColor(img, img, CV_BGR2GRAY);

		// downsample the image
		cv::pyrDown(img, img, cv::Size(img.cols / 2, img.rows / 2));
		cv::pyrDown(frame, original, cv::Size(frame.cols / 2, frame.rows / 2));

		// equalize the image
		equalizeHist(img, img);
		std::vector<cv::Rect> found, found_filtered;

		if (lastHumanDetectionBox.area() > 0) {
			cv::Rect r = lastHumanDetectionBox;
			int left_boundary, right_boundary;
			left_boundary = std::max(r.x - 50, 0);
			right_boundary = std::min(r.x + r.width + 50, img.cols);

			cv::Rect rec(left_boundary, 0, right_boundary - left_boundary, img.rows);

			cv::Mat Roi = img(rec);

			humanHOG.detectMultiScale(Roi, found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

			size_t i, j;
			for (i = 0; i<found.size(); i++) {
				cv::Rect r = found[i];
				for (j = 0; j<found.size(); j++) {
					if (j != i && (r & found[j]) == r) {
						break;
					}
				}
				if (j == found.size()) {
					found_filtered.push_back(r);
				}
			}

			cv::Rect max_rect;
			max_rect = find_max_rec(found_filtered);

			if (max_rect.area()>0) {

				max_rect.x += cvRound(max_rect.width*0.1);
				max_rect.width = cvRound(max_rect.width*0.8);
				max_rect.y += cvRound(max_rect.height*0.06);
				max_rect.height = cvRound(max_rect.height*0.9);
				cv::Rect WhereRec(left_boundary + max_rect.x, max_rect.y, max_rect.width, max_rect.height);

				rectangle(original, WhereRec, cv::Scalar(0, 255, 0), 2);
			}

			//copy the found filter
			lastHumanDetectionBox = max_rect;
		} else {
			humanHOG.detectMultiScale(img, found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

			size_t i, j;
			for (i = 0; i<found.size(); i++) {
				cv::Rect r = found[i];
				for (j = 0; j<found.size(); j++) {
					if (j != i && (r & found[j]) == r) {
						break;
					}
				}
				if (j == found.size()) {
					found_filtered.push_back(r);
				}
			}

			cv::Rect max_rect;
			max_rect = find_max_rec(found_filtered);

			if (max_rect.area()>0) {
				max_rect.x += cvRound(max_rect.width*0.1);
				max_rect.width = cvRound(max_rect.width*0.8);
				max_rect.y += cvRound(max_rect.height*0.06);
				max_rect.height = cvRound(max_rect.height*0.9);
				rectangle(original, max_rect.tl(), max_rect.br(), cv::Scalar(0, 255, 0), 2);

			}

			//copy the found filter
			lastHumanDetectionBox = max_rect;
		}
		imshow("original", original);
	}

	void HumanDetector::detectBodyPose(const cv::Mat& frame) {
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

	void HumanDetector::detectHeadPose(const cv::Mat& frame) {
		cv::CascadeClassifier faceDetector("C:\\opencv\\data\\haarcascades\\haarcascade_frontalface_alt2.xml");

		// Create an instance of Facemark
		cv::Ptr<cv::face::Facemark> facemark = cv::face::FacemarkLBF::create();

		// Load landmark detector
		facemark->loadModel("C:\\dev\\OpenARK_dataset\\lbfmodel.yaml");

		cv::VideoCapture cam(0);
		cv::Mat gray;

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
			//Visualizer::visualizeFaceLandmarks(frame, landmarks[0]);

			image_points.push_back(landmarks[0][30]);    // Nose tip
			image_points.push_back(landmarks[0][8]);     // Chin
			image_points.push_back(landmarks[0][36]);    // Left eye left corner
			image_points.push_back(landmarks[0][45]);    // Right eye right corner
			image_points.push_back(landmarks[0][60]);    // Left Mouth corner
			image_points.push_back(landmarks[0][64]);    // Right mouth corner
		}
		else {
			cv::imshow("Facial Landmark Detection", frame);
			return;
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

	cv::Rect HumanDetector::find_max_rec(const std::vector<cv::Rect>& found_filtered) {
		int max_size = 0;
		cv::Rect max_rect;
		for (int i = 0; i<found_filtered.size(); i++) {
			cv::Rect r = found_filtered[i];
			if (r.area()>max_size) {
				max_rect = found_filtered[i];
			}

		}
		return max_rect;
	}
}