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
		// std::string filename = "..//OpenARK_Datasets//HandDataSet1//img" + std::to_string(frame) + ".yml";
		// if (!camera->readImage(filename))
			// break;

		// Show XYZ map
		cv::imshow("XYZ Map", Visualizer::visualizeXYZMap(camera->getXYZMap()));

		// Find hands
        auto hands = camera->queryHands();
        if (!hands.empty()){
            Hand hand = hands[0].getHand();

            // Show the hand
            cv::imshow("Hand", Visualizer::visualizeHand(camera->getXYZMap(), hand.pointer_finger_ij, hand.shape_centroid_ij));
        }

		/**** Start: Loop Break Condition ****/
		int c = cv::waitKey(1);
		if (c == 'q' || c == 'Q' || c == 27) {
			break;
		}
		/**** End: Loop Break Condition ****/

		++frame;
	}

	camera->destroyInstance();
	return 0;
}