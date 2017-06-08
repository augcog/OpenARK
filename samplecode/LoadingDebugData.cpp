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
#include "Util.h"

int main() {
	DepthCamera* pmd = new PMDCamera(false); // Initiatize with use_live_sensor = false
	int frame = 0;

	while (true)
	{
		// Read in each individual frame from file
		std::string filename = "..//OpenARK_Datasets//HandDataSet2//img" + std::to_string(frame) + ".yml";
		if (!pmd->readImage(filename))
			break;

		// Remove nosie from the frame
		pmd->removeNoise();

		// Display the resultant image
		cv::imshow("XYZ Map", Visualizer::visualizeXYZMap(pmd->getXYZMap()));

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