// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "SR300Camera.h"
#include "Visualizer.h"
#include "Hand.h"
#include "Plane.h"
#include "Util.h"

using namespace ark;

int main() {
	DepthCamera* camera = new SR300Camera();
	int frame = 0;

	while (true)
	{
        camera->nextFrame();
		// // Load in the debug data
		// std::string filename = "..//OpenARK_Datasets//ClusterDataSet1//img" + std::to_string(frame) + ".yml";
		// if (!camera->readImage(filename))
			// break;

		// Show image
		cv::imshow("XYZ Map", Visualizer::visualizeXYZMap(camera->getXYZMap()));

		// // Find the plane
		// Plane plane = Plane::Plane(camera->getXYZMap());
		// Visualizer::visualizeCloud(plane.getDownCloud());
		// cv::imshow("Plane Regression", Visualizer::visualizePlaneRegression(camera->getXYZMap(), plane.getPlaneEquation(), plane.R_SQUARED_DISTANCE_THRESHOLD));

		// // Remove the plane
		// camera->removePoints(plane.getPlaneIndicies());
		// cv::imshow("Hand Map", Visualizer::visualizeXYZMap(camera->getXYZMap()));
		// camera->computeClusters(0.02, 500);

		// Find the right hand
        auto hands = camera->queryHands();
		Hand right_hand = hands[0].getHand();

		// Show the hand
		cv::imshow("Hand", Visualizer::visualizeHand(camera->getXYZMap(), right_hand.pointer_finger_ij, right_hand.shape_centroid_ij));
		// if (right_hand.touchObject(plane.getPlaneEquation(), plane.R_SQUARED_DISTANCE_THRESHOLD * 3) == true)
		// {
			// printf("Touched Plane\n");
		// }


		/**** Start: Loop Break Condition ****/
		int c = cvWaitKey(1);
		if (c == 'q' || c == 'Q' || c == 27) {
			break;
		}
		/**** End: Loop Break Condition ****/

		frame++;
	}

	camera->destroyInstance();
	return 0;
}