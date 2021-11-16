// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>
#include <memory>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "util/Core.h"
#include "camera/SR300Camera.h"
#include "util/Visualizer.h"

using namespace ark;

int main() {
    // initialize
    ark::DepthCamera::Ptr camera = std::make_shared<ark::SR300Camera>();
    ark::DetectionParams::Ptr params = ark::DetectionParams::create();
    ark::PlaneDetector planeDetector(params);

    // use the plane detector to remove planes and detect contact points
    ark::HandDetector handDetector(planeDetector, params);

    camera->beginCapture();

    int frame = 0;
    while (true)
    {
        // Show image
        cv::Mat xyzVisual; ark::Visualizer::visualizeXYZMap(camera->getXYZMap(), xyzVisual);
        cv::imshow("XYZ Map", xyzVisual);

        // Update detectors
        planeDetector->update(*camera);
        handDetector->update(*camera);

        // Detect objects in frame
        auto planes = planeDetector->getPlanes();
        auto hands = handDetector->getHands();

        // Find the hand assigned the highest confidence
        ark::Hand::Ptr primary_hand = hands[0];

        // Show the hand, along with the SVM confidence and contact points
        cv::Mat visual = camera->getXYZMap().clone();
        ark::Visualizer::visualizeHand(visual, visual, primary_hand.get(), primary_hand->getSVMConfidence(), &planes);
        cv::imshow("Result", visual);

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
