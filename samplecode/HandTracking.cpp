// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "util/Core.h"
#include "camera/SR300Camera.h"
#include "util/Visualizer.h"

using namespace ark;

int main() {
    // change the SR300Camera to other type if needed
    ark::DepthCamera::Ptr camera = std::make_shared<ark::SR300Camera>();

    ark::DetectionParams::Ptr params = ark::DetectionParams::create();
    ark::HandDetector handDetector(false, params); // change first argument to true to remove planes in frame

    camera->beginCapture();

    int frame = 0;
    while (true)
    {
        // Show image
        cv::Mat xyzVisual; ark::Visualizer::visualizeXYZMap(camera->getXYZMap(), xyzVisual);
        cv::imshow("XYZ Map", xyzVisual);

        // Find hands
        handDetector->update(*camera);
        auto hands = handDetector->getHands();

        if (!hands.empty()){
            ark::Hand::Ptr hand = hands[0];

            // Show the hand
            cv::Mat visual = camera->getXYZMap().clone();
            ark::Visualizer::visualizeHand(visual, visual, hand.get(), hand->getSVMConfidence());
            cv::imshow("Result", visual);
        }

        /**** Start: Loop Break Condition ****/
        int c = cv::waitKey(1);
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }
        /**** End: Loop Break Condition ****/

        ++frame;
    }
    return 0;
}
