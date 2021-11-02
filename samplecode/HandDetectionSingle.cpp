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

using namespace ark;

int main(int argc, char * argv[]) {
    if (argc < 2) {
        printf("Please specify the input image file\n");
        return 0;
    }

    cv::Mat input = cv::imread(argv[1]);
    ark::HandDetector handDetector(false); // change first argument to true to remove planes in frame

    // Find hands
    handDetector->update(input);
    auto hands = handDetector->getHands();

    if (!hands.empty()){
        ark::Hand::Ptr hand = hands[0];
        printf("Found hand. Showing visualization...\n");

        // Show the hand
        cv::Mat visual = input.clone();
        ark::Visualizer::visualizeHand(visual, visual, hand.get(), hand->getSVMConfidence());
        cv::imshow("Result", visual);
    }
    else {
        printf("No hands found!\n");
    }

    return 0;
}
