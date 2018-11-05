#include "HumanDetector.h"

namespace ark {
	const std::vector<std::pair<int, int>> mapIdx = {
		{0,1}, {2,3}, {4,5}, {6,7}, {8,9}, {10,11},
		{12,13}, {14,15}, {16,17}, {18,19}, {20,21},
		{22,23}, {24,25}, {26,27}
	};

	const std::vector<std::pair<int, int>> posePairs = {
		{ 0,1 },{ 1,2 },{ 2,3 },
		{ 3,4 },{ 1,5 },{ 5,6 },
		{ 6,7 },{ 1,14 },{ 14,8 },{ 8,9 },
		{ 9,10 },{ 14,11 },{ 11,12 },{ 12,13 }
	};


	HumanDetector::HumanDetector(DetectionParams::Ptr params) {
		// Since we have seen no humans previously, we set this to default value
		lastHumanDetectionBox = cv::Rect(0, 0, 0, 0);

		// Load the human HOG descriptor
		humanHOG.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

		// Load the OpenPose model
		openPoseNet = cv::dnn::readNetFromCaffe(MPII_PROTO_FILE_PATH, MPII_WEIGHTS_FILE_PATH);

		// Load face models
		facemark = cv::face::FacemarkLBF::create();
		facemark->loadModel("C:\\dev\\OpenARK_dataset\\lbfmodel.yaml");
		faceDetector.load("C:\\opencv\\data\\haarcascades\\haarcascade_frontalface_alt2.xml");

		face_3D_model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
		face_3D_model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
		face_3D_model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
		face_3D_model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
		face_3D_model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
		face_3D_model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner
	}

	void HumanDetector::detect(cv::Mat & image) {
		// Get the human area via HOG
		//detectHumanHOG(image);

		// Feed the HOG area into the Body Pose Estimation
		detectBodyPose(image);

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
		//imshow("original", original);
	}

	void HumanDetector::detectBodyPose(const cv::Mat& frame) {
		const int nPoints = 15; // ignore 'background' point

		// Prepare the frame to be fed to the network
		cv::Mat inpBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, frame.size(), cv::Scalar(0, 0, 0));

		// Set the prepared object as the input blob of the network
		openPoseNet.setInput(inpBlob);

		cv::Mat netOutputBlob = openPoseNet.forward();
		std::vector<cv::Mat> netOutputParts;
		splitNetOutputBlobToParts(netOutputBlob, cv::Size(frame.cols, frame.rows), netOutputParts);

		int keyPointId = 0;
		std::vector<std::vector<KeyPoint>> detectedKeypoints;
		std::vector<KeyPoint> keyPointsList;

		for (int i = 0; i < nPoints; ++i) {
			std::vector<KeyPoint> keyPoints;

			getKeyPoints(netOutputParts[i], 0.3, keyPoints);

			for (int i = 0; i< keyPoints.size(); ++i, ++keyPointId) {
				keyPoints[i].id = keyPointId;
			}

			detectedKeypoints.push_back(keyPoints);
			keyPointsList.insert(keyPointsList.end(), keyPoints.begin(), keyPoints.end());
		}

		std::vector<cv::Scalar> colors;
		populateColorPalette(colors, nPoints);

		cv::Mat outputFrame = frame.clone();

		for (int i = 0; i < nPoints; ++i) {
			for (int j = 0; j < detectedKeypoints[i].size(); ++j) {
				cv::circle(outputFrame, detectedKeypoints[i][j].point, 5, colors[i], -1, cv::LINE_AA);
			}
		}

		std::vector<std::vector<ValidPair>> validPairs;
		std::set<int> invalidPairs;
		getValidPairs(netOutputParts, detectedKeypoints, validPairs, invalidPairs);

		std::vector<std::vector<int>> personwiseKeypoints;
		getPersonwiseKeypoints(validPairs, invalidPairs, personwiseKeypoints);

		for (int i = 0; i < personwiseKeypoints.size(); i++) {
			std::shared_ptr<HumanBody> human = std::make_shared<HumanBody>();
			human_bodies.push_back(human);
		}

		for (int n = 0; n < personwiseKeypoints.size(); ++n) {
			human_bodies[n]->MPIISkeleton2D.clear();
			human_bodies[n]->MPIISkeleton2D.resize(nPoints);
			for (int i = 0; i < nPoints; ++i) {
				int indexA = personwiseKeypoints[n][i];

				if (indexA == -1) {
					continue;
				}

				const KeyPoint& kpA = keyPointsList[indexA];

				cv::circle(outputFrame, kpA.point, 2, cv::Scalar(255, 0, 0), 2);
				human_bodies[n]->MPIISkeleton2D[i] = kpA.point;
				
			}
		}

		//cv::imshow("Detected Pose", outputFrame);
	}

	void HumanDetector::getPersonwiseKeypoints(const std::vector<std::vector<ValidPair>>& validPairs,
		const std::set<int>& invalidPairs,
		std::vector<std::vector<int>>& personwiseKeypoints) {
		for (int k = 0; k < mapIdx.size(); ++k) {
			if (invalidPairs.find(k) != invalidPairs.end()) {
				continue;
			}

			const std::vector<ValidPair>& localValidPairs(validPairs[k]);

			int indexA(posePairs[k].first);
			int indexB(posePairs[k].second);

			for (int i = 0; i< localValidPairs.size(); ++i) {
				bool found = false;
				int personIdx = -1;

				for (int j = 0; !found && j < personwiseKeypoints.size(); ++j) {
					if (indexA < personwiseKeypoints[j].size() &&
						personwiseKeypoints[j][indexA] == localValidPairs[i].aId) {
						personIdx = j;
						found = true;
					}
				}/* j */

				if (found) {
					personwiseKeypoints[personIdx].at(indexB) = localValidPairs[i].bId;
				}
				else if (k < 15) {
					std::vector<int> lpkp(std::vector<int>(18, -1));

					lpkp.at(indexA) = localValidPairs[i].aId;
					lpkp.at(indexB) = localValidPairs[i].bId;

					personwiseKeypoints.push_back(lpkp);
				}

			}/* i */
		}/* k */
	}

	void HumanDetector::getValidPairs(const std::vector<cv::Mat>& netOutputParts,
		const std::vector<std::vector<KeyPoint>>& detectedKeypoints,
		std::vector<std::vector<ValidPair>>& validPairs,
		std::set<int>& invalidPairs) {

		int nInterpSamples = 10;
		float pafScoreTh = 0.1;
		float confTh = 0.7;

		for (int k = 0; k < mapIdx.size(); ++k) {

			//A->B constitute a limb
			cv::Mat pafA = netOutputParts[16 + mapIdx[k].first];
			cv::Mat pafB = netOutputParts[16 + mapIdx[k].second];

			//Find the keypoints for the first and second limb
			const std::vector<KeyPoint>& candA = detectedKeypoints[posePairs[k].first];
			const std::vector<KeyPoint>& candB = detectedKeypoints[posePairs[k].second];

			int nA = candA.size();
			int nB = candB.size();

			/*
			# If keypoints for the joint-pair is detected
			# check every joint in candA with every joint in candB
			# Calculate the distance vector between the two joints
			# Find the PAF values at a set of interpolated points between the joints
			# Use the above formula to compute a score to mark the connection valid
			*/

			if (nA != 0 && nB != 0) {
				std::vector<ValidPair> localValidPairs;

				for (int i = 0; i< nA; ++i) {
					int maxJ = -1;
					float maxScore = -1;
					bool found = false;

					for (int j = 0; j < nB; ++j) {
						std::pair<float, float> distance(candB[j].point.x - candA[i].point.x, candB[j].point.y - candA[i].point.y);

						float norm = std::sqrt(distance.first*distance.first + distance.second*distance.second);

						if (!norm) {
							continue;
						}

						distance.first /= norm;
						distance.second /= norm;

						//Find p(u)
						std::vector<cv::Point> interpCoords;
						populateInterpPoints(candA[i].point, candB[j].point, nInterpSamples, interpCoords);
						//Find L(p(u))
						std::vector<std::pair<float, float>> pafInterp;
						for (int l = 0; l < interpCoords.size(); ++l) {
							pafInterp.push_back(
								std::pair<float, float>(
									pafA.at<float>(interpCoords[l].y, interpCoords[l].x),
									pafB.at<float>(interpCoords[l].y, interpCoords[l].x)
									));
						}

						std::vector<float> pafScores;
						float sumOfPafScores = 0;
						int numOverTh = 0;
						for (int l = 0; l< pafInterp.size(); ++l) {
							float score = pafInterp[l].first*distance.first + pafInterp[l].second*distance.second;
							sumOfPafScores += score;
							if (score > pafScoreTh) {
								++numOverTh;
							}

							pafScores.push_back(score);
						}

						float avgPafScore = sumOfPafScores / ((float)pafInterp.size());

						if (((float)numOverTh) / ((float)nInterpSamples) > confTh) {
							if (avgPafScore > maxScore) {
								maxJ = j;
								maxScore = avgPafScore;
								found = true;
							}
						}

					}/* j */

					if (found) {
						localValidPairs.push_back(ValidPair(candA[i].id, candB[maxJ].id, maxScore));
					}

				}/* i */

				validPairs.push_back(localValidPairs);

			}
			else {
				invalidPairs.insert(k);
				validPairs.push_back(std::vector<ValidPair>());
			}
		}/* k */
	}

	void HumanDetector::populateInterpPoints(const cv::Point& a, const cv::Point& b, int numPoints, std::vector<cv::Point>& interpCoords) {
		float xStep = ((float)(b.x - a.x)) / (float)(numPoints - 1);
		float yStep = ((float)(b.y - a.y)) / (float)(numPoints - 1);

		interpCoords.push_back(a);

		for (int i = 1; i< numPoints - 1; ++i) {
			interpCoords.push_back(cv::Point(a.x + xStep*i, a.y + yStep*i));
		}

		interpCoords.push_back(b);
	}

	void HumanDetector::populateColorPalette(std::vector<cv::Scalar>& colors, int nColors) {
		//std::random_device rd;
		//std::mt19937 gen(rd());
		//std::uniform_int_distribution<> dis1(64, 200);
		//std::uniform_int_distribution<> dis2(100, 255);
		//std::uniform_int_distribution<> dis3(100, 255);

		for (int i = 0; i < nColors; ++i) {
			colors.push_back(cv::Scalar(100,100,100));
		}
	}

	void HumanDetector::splitNetOutputBlobToParts(cv::Mat& netOutputBlob, const cv::Size& targetSize, std::vector<cv::Mat>& netOutputParts) {
		int nParts = netOutputBlob.size[1];
		int h = netOutputBlob.size[2];
		int w = netOutputBlob.size[3];

		for (int i = 0; i< nParts; ++i) {
			cv::Mat part(h, w, CV_32F, netOutputBlob.ptr(0, i));

			cv::Mat resizedPart;

			cv::resize(part, resizedPart, targetSize);

			netOutputParts.push_back(resizedPart);
		}
	}

	void HumanDetector::getKeyPoints(cv::Mat& probMap, double threshold, std::vector<KeyPoint>& keyPoints) {
		cv::Mat smoothProbMap;
		cv::GaussianBlur(probMap, smoothProbMap, cv::Size(3, 3), 0, 0);

		cv::Mat maskedProbMap;
		cv::threshold(smoothProbMap, maskedProbMap, threshold, 255, cv::THRESH_BINARY);

		maskedProbMap.convertTo(maskedProbMap, CV_8U, 1);

		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(maskedProbMap, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		for (int i = 0; i < contours.size(); ++i) {
			cv::Mat blobMask = cv::Mat::zeros(smoothProbMap.rows, smoothProbMap.cols, smoothProbMap.type());

			cv::fillConvexPoly(blobMask, contours[i], cv::Scalar(1));

			double maxVal;
			cv::Point maxLoc;

			cv::minMaxLoc(smoothProbMap.mul(blobMask), 0, &maxVal, 0, &maxLoc);

			keyPoints.push_back(KeyPoint(maxLoc, probMap.at<float>(maxLoc.y, maxLoc.x)));
		}
	}

	void HumanDetector::detectHeadPose(const cv::Mat& frame) {
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

																				// Camera internals
		double focal_length = frame.cols; // Approximate focal length.
		Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
		cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
		cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

																				// Output rotation and translation
		cv::Mat rotation_vector; // Rotation in axis-angle form
		cv::Mat translation_vector;

		// Solve for pose
		cv::solvePnP(face_3D_model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);


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

	std::vector<std::shared_ptr<HumanBody>> HumanDetector::getHumanBodies() {
		return human_bodies;
	}
}