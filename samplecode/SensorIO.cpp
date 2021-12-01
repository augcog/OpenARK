// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>

// OpenCV Libraries #include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "openark/util/Core.h"
#include "openark/camera/SR300Camera.h"
#include "openark/util/Visualizer.h"

int main() {
    ark::DepthCamera::Ptr camera = std::make_shared<ark::SR300Camera>();

    // turn on camera
    camera->beginCapture();

    int frame = 0;
    while (true)
    {
        // Show image
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
