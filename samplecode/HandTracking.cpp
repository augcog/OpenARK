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
		std::string filename = "..//OpenARK_Datasets//HandDataSet1//img" + std::to_string(frame) + ".yml";
		if (!pmd->readImage(filename))
			break;
		
		// Remove noise on the frame
		pmd->removeNoise();
		cv::imshow("XYZ Map", Visualizer::visualizeXYZMap(pmd->getXYZMap()));
		
		// Find the right hand
		Hand right_hand = Hand(pmd->getClusters());
		
		// Show the hand
		cv::imshow("Hand", Visualizer::visualizeHand(pmd->getXYZMap(), right_hand.pointer_finger_ij, right_hand.shape_centroid_ij));
		

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