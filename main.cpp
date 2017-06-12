// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>

// OpenCV Libraries
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "PMDCamera.h"
#include "SR300Camera.h"
#include "Webcam.h"
#include "Visualizer.h"
#include "Hand.h"
#include "Plane.h"
#include "Calibration.h"
#include "Util.h"
#include "UDPSender.h"
#include "Object3D.h"
#include "StreamingAverager.h"
#include "global.h"

int main() {
	clock_t starttime = clock();
	DepthCamera * camera = nullptr;
	if (camera_name == "pmd") {
		camera = new PMDCamera();
	}
	else if (camera_name == "sr300") {
		camera = new SR300Camera();
	}

	//RGBCamera *cam = new Webcam(1);
	int frame = 0;
	//Calibration::XYZToUnity(*pmd, 4, 4, 3);
	cv::FileStorage fs;
	fs.open("RT_Transform.txt", cv::FileStorage::READ);

	cv::Mat r, t;
	fs["R"] >> r;
	fs["T"] >> t;

	fs.release();

	UDPSender u = UDPSender();
	cv::namedWindow("Results", CV_WINDOW_NORMAL);

	StreamingAverager handAverager = StreamingAverager(4, 0.1);
	StreamingAverager paleeteAverager = StreamingAverager(6, 0.05);

	while (true)
	{
		camera->update();

		/**
		std::string filename = "..//OpenARK_Datasets//TwoHandDataSet1//img" + std::to_string(frame) + ".yml";
		if (!pmd->readImage(filename))
		break;
		**/

		// Loading image from sensor
		camera->removeNoise();
		if (camera->badInput) {
			cv::waitKey(10);
			continue;
		}

		// Classifying objects in the scene
	    camera->computeClusters(0.02, 500);
		std::vector<cv::Mat> clusters = camera->getClusters();
		std::vector<Object3D> objects;
		int handObjectIndex = -1, planeObjectIndex = -1;
		for (int i = 0; i < clusters.size(); i++) {
			Object3D obj = Object3D(clusters[i].clone());
			if (obj.hasHand) {
				handObjectIndex = i;
			}

			if (obj.hasPlane) {
				planeObjectIndex = i;
			}
			objects.push_back(obj);
		}

		// Interprate the relationship between the objects
		bool clicked = false, paletteFound = false;
		Object3D handObject, planeObject;
		cv::Point paletteCenter(-1. - 1);
		cv::Mat mask = cv::Mat::zeros(camera->getXYZMap().rows, camera->getXYZMap().cols, CV_8UC1);
		if (planeObjectIndex != -1 && handObjectIndex != -1) {
			planeObject = objects[planeObjectIndex];
			handObject = objects[handObjectIndex];

			clicked = handObject.getHand().touchObject(planeObject.getPlane().getPlaneEquation(), planeObject.getPlane().R_SQUARED_DISTANCE_THRESHOLD * 5);
			cv::Mat scene = Visualizer::visualizePlaneRegression(camera->getXYZMap(), planeObject.getPlane().getPlaneEquation(), planeObject.getPlane().R_SQUARED_DISTANCE_THRESHOLD, clicked);
			//scene = Visualizer::visualizeHand(scene, handObject.getHand().pointer_finger_ij, handObject.getHand().shape_centroid_ij);
			if (planeObject.leftEdgeConnected) {
				Visualizer::visualizePlanePoints(mask, planeObject.getPlane().getPlaneIndicies());
				cv::Moments m = cv::moments(mask, false);
				paletteCenter = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
				cv::circle(scene, paletteCenter, 2, cv::Scalar(0, 0, 255), 2);
				paletteFound = true;
			}
			cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
			cv::imshow("Results", scene);
		}
		else if (handObjectIndex != -1) {
			handObject = objects[handObjectIndex];
			//cv::imshow("Results", Visualizer::visualizeHand(pmd->getXYZMap(), handObject.getHand().pointer_finger_ij, handObject.getHand().shape_centroid_ij));
		}
		else if (planeObjectIndex != -1) {
			planeObject = objects[planeObjectIndex];
			cv::Mat scene = Visualizer::visualizePlaneRegression(camera->getXYZMap(), planeObject.getPlane().getPlaneEquation(), planeObject.getPlane().R_SQUARED_DISTANCE_THRESHOLD, clicked);
			if (planeObject.leftEdgeConnected) {
				Visualizer::visualizePlanePoints(mask, planeObject.getPlane().getPlaneIndicies());
				cv::Moments m = cv::moments(mask, false);
				paletteCenter = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
				cv::circle(scene, paletteCenter, 2, cv::Scalar(0, 0, 255), 2);
				paletteFound = true;
			}
			cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
			cv::imshow("Results", scene);
		}

		// Organize the data and send to game engine
		std::string handX = "-", handY = "-", handZ = "-";
		std::string paletteX = "-", paletteY = "-", paletteZ = "-";
		std::string clickStatus = "2";
		std::string num_fingers = "0";
		if (handObjectIndex != -1) {
			cv::Vec3f handPos = handAverager.addDataPoint(objects[handObjectIndex].getHand().fingers_xyz[0]);
			//float hand_pt[3] = { objects[handObjectIndex].getHand().pointer_finger_xyz[0], objects[handObjectIndex].getHand().pointer_finger_xyz[1], objects[handObjectIndex].getHand().pointer_finger_xyz[2]};
			float hand_pt[3] = { handPos[0], handPos[1], handPos[2] };
			cv::Mat hand_mat = cv::Mat(3, 1, CV_32FC1, &hand_pt);
			hand_mat = r*hand_mat + t;
			handX = std::to_string(hand_mat.at<float>(0, 0));
			handY = std::to_string(hand_mat.at<float>(1, 0));
			handZ = std::to_string(hand_mat.at<float>(2, 0));
			num_fingers = std::to_string(objects[handObjectIndex].getHand().fingers_xyz.size());
		}
		else {
			handAverager.addEmptyPoint();
		}
		if (paletteFound) {
			cv::Vec3f pt = paleeteAverager.addDataPoint(camera->getXYZMap().at<cv::Vec3f>(paletteCenter.y, paletteCenter.x));
			float palette_pt[3] = { pt[0], pt[1], pt[2] };
			cv::Mat palette_mat = cv::Mat(3, 1, CV_32FC1, &palette_pt);
			palette_mat = r*palette_mat + t;
			paletteX = std::to_string(palette_mat.at<float>(0, 0));
			paletteY = std::to_string(palette_mat.at<float>(1, 0));
			paletteZ = std::to_string(palette_mat.at<float>(2, 0));
		}
		else {
			paleeteAverager.addEmptyPoint();
		}
		if (clicked) {
			clickStatus = "1";
		}

		std::string tempS = "";
		tempS = handX + "%" + handY + "%" + handZ + "%" + paletteX + "%" + paletteY + "%" + paletteZ + "%" + clickStatus + "%" + num_fingers;
		u.send(tempS);



		/**** Start: Write Frames to File ****/
		//std::string filename = "img" + std::to_string(frame) + ".yml";
		//pmd->writeImage(filename);
		//std::cout << filename << std::endl;
		/**** End: Write Frames to File ****/

		/**** Start: Loop Break Condition ****/
		int c = cv::waitKey(1);
		if (c == 'q' || c == 'Q' || c == 27) {
			break;
		}
		/**** End: Loop Break Condition ****/
		frame++;
	}

	camera->destroyInstance();
	cv::destroyAllWindows();
	return 0;
}