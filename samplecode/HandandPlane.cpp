// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "PMDCamera.h"
#include "Visualizer.h"
#include "Hand.h"
#include "Plane.h"
#include "Util.h"

int main() {
	DepthCamera* pmd = new PMDCamera(false);
	int frame = 0;
	
	while (true)
	{
		// Load in the debug data
		std::string filename = "..//OpenARK_Datasets//ClusterDataSet1//img" + std::to_string(frame) + ".yml";
		if (!pmd->readImage(filename))
			break;
		
		// Remove noise on the frame
		pmd->removeNoise();
		cv::imshow("XYZ Map", Visualizer::visualizeXYZMap(pmd->getXYZMap()));
		
		// Find the plane
		Plane plane = Plane::Plane(pmd->getXYZMap());
		Visualizer::visualizeCloud(plane.getDownCloud());
		cv::imshow("Plane Regression", Visualizer::visualizePlaneRegression(pmd->getXYZMap(), plane.getPlaneEquation(), plane.R_SQUARED_DISTANCE_THRESHOLD));
		
		// Remove the plane
		pmd->removePoints(plane.getPlaneIndicies());
		cv::imshow("Hand Map", Visualizer::visualizeXYZMap(pmd->getXYZMap()));
		pmd->computeClusters(0.02, 500);
		
		// Find the right hand
		Hand right_hand = Hand(pmd->getClusters());
		
		// Show the hand
		cv::imshow("Hand", Visualizer::visualizeHand(pmd->getXYZMap(), right_hand.pointer_finger_ij, right_hand.shape_centroid_ij));
		if (right_hand.touchObject(plane.getPlaneEquation(), plane.R_SQUARED_DISTANCE_THRESHOLD * 3) == true)
		{
			printf("Touched Plane\n");
		}
		

		/**** Start: Loop Break Condition ****/
		int c = cvWaitKey(1);
		if (c == 'q' || c == 'Q' || c == 27) {
			break;
		}
		/**** End: Loop Break Condition ****/

		frame++;
	}

	pmd->destroyInstance();
	return 0;
}