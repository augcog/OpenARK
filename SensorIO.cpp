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
	DepthCamera* pmd = new PMDCamera();
	int frame = 0;
	
	while (true)
	{
		// Update the current frame
		pmd->update();

		// REmove the current frame
		pmd->removeNoise();

		// Visualize the XYZ Map
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