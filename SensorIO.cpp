#include "stdafx.h"

// OpenARK Libraries
#include "SR300Camera.h"
#include "Visualizer.h"
#include "Util.h"

using namespace ark;

int main() {
	DepthCamera* camera = new SR300Camera();
	int frame = 0;

	while (true)
	{
		// Update the current frame
		camera->nextFrame();

		// Visualize the XYZ Map
		cv::imshow("XYZ Map", Visualizer::visualizeXYZMap(camera->getXYZMap()));

		/**** Start: Loop Break Condition ****/
		int c = cv::waitKey(1);
		if (c == 'q' || c == 'Q' || c == 27) {
			break;
		}
		/**** End: Loop Break Condition ****/
		frame++;
	}

	camera->destroyInstance();
	return 0;
}