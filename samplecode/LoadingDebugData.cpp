// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "util/Core.h"
#include "SR300Camera.h"
#include "util/Visualizer.h"

int main() {
    ark::DepthCamera::Ptr camera = std::make_shared<ark::SR300Camera>();
    camera->beginCapture();

    int frame = 0;
    while (true)
    {
        // Read in each individual frame from file
        std::string filename = "..//OpenARK_Datasets//HandDataSet2//img" + std::to_string(frame) + ".yml";
        if (!camera->readImage(filename))
            break;

        // Display the resultant image
        cv::Mat xyzVisual; ark::Visualizer::visualizeXYZMap(camera->getXYZMap(), xyzVisual);
        cv::imshow("XYZ Map", xyzVisual);

        /**** Start: Loop Break Condition ****/
        int c = cv::waitKey(1);
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }
        /**** End: Loop Break Condition ****/

        frame++;
    }

    return 0;
}
