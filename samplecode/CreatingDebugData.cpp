// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>
#include <memory>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "openark/util/Core.h"
#include "openark/camera/SR300Camera.h"
#include "openark/util/Visualizer.h"

int main() {
    ark::DepthCamera::Ptr camera = std::make_shared<ark::SR300Camera>();
    camera->beginCapture();

    int frame = 0;
    while (true)
    {
        /**** Start: Write Frames to File ****/
        std::string filename = "img" + std::to_string(frame) + ".yml";
        camera->writeImage(filename);
        std::cout << "Saved: " << filename <<  std::endl;
        /**** End: Write Frames to File ****/
        cv::Mat visual; ark::Visualizer::visualizeXYZMap(camera->getXYZMap(), visual);
        cv::imshow("XYZ Map", visual);

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
