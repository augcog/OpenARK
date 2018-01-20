#include "stdafx.h"

// OpenARK Libraries
#include "version.h"
#ifdef PMDSDK_ENABLED
    #include "PMDCamera.h"
#endif
#ifdef RSSDK_ENABLED
    #include "SR300Camera.h"
#endif

#include "Webcam.h"
#include "Visualizer.h"
#include "Hand.h"
#include "Plane.h"
#include "Calibration.h"
#include "Util.h"
#include "UDPSender.h"
#include "Object3D.h"
#include "StreamingAverager.h"

#include "HandFeatureExtractor.h"
#include "HandClassifier.h"

using namespace ark;

static inline void drawHand(cv::Mat & image, Object3D & obj, float confidence = 0.0) {
    if (obj.getContour().size() > 2) {
        cv::polylines(image, obj.getContour(), true, cv::Scalar(0, 255, 0), 1);
    } 
       
    if (obj.hasHand) {
        image = Visualizer::visualizeHand(image, obj.getHand());
    }

    std::stringstream disp;
    disp << std::setprecision(3) << std::fixed << confidence;
    Point2i center = obj.getCenterIj(), 
            dispPt = center - Point2i((int)disp.str().size() * 8, 0);
    cv::putText(image, disp.str(), dispPt, 0, 0.8, cv::Scalar(255, 255, 255), 1);
}

int main() {
    printf("Welcome to OpenARK v %s\n", ark::VERSION);
    DepthCamera * camera;

#ifdef PMDSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "pmd")) {
        camera = new PMDCamera();
    }
    else {
        return -1;
    }
#endif
#ifdef RSSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "sr300")) {
        camera = new SR300Camera();
    }
    else {
        return -1;
    }
#endif
    // seed rng
    srand(time(NULL));

    // store frame & FPS information
    int frame = 0;
    clock_t cycleStartTime = 0;
    const int FPS_FRAMES = 5;
    float currFps = 0;

    // main loop
    while (true)
    {
        // go to next frame
        camera->nextFrame();

        #ifdef DEMO
            // show depth image
            if (camera->getXYZMap().rows > 0) {
                cv::imshow("OpenARK Depth Image", camera->getXYZMap());
                if (camera->hasRGBImage()) {
                    cv::imshow("OpenARK Color Image", camera->getRGBImage());
                }
            }
        #endif

        // classify objects in the scene
        std::vector<Object3D> objects = camera->queryObjects();

        // create visualization of hand objects
        cv::Mat handVisual;

#ifdef DEMO
        if (camera->hasIRImage()) {
            camera->getIRImage().convertTo(handVisual, CV_8UC1);
            handVisual /= 2;
            cv::cvtColor(handVisual, handVisual, cv::COLOR_GRAY2BGR, 3);
        }
        else {
#endif
            handVisual = cv::Mat::zeros(camera->getHeight(), camera->getWidth(), CV_8UC3);
#ifdef DEMO
        }
#endif

        int handObjectIndex = -1, handCount = 0, planeObjectIndex = -1;

        // object is immediately considered to be hand if SVM confidence is above this
        static const double SVM_HIGH_CONFIDENCE_THRESH = 0.59;

        float bestHandDist = FLT_MAX, bestDistConf = 0;

        for (int i = 0; i < objects.size(); ++i) {  
            Object3D & obj = objects[i];

            if (obj.hasPlane) {
                planeObjectIndex = i;
            }

            if (obj.hasHand) {
                float distance = obj.getDepth();

                double conf = obj.getHand().svm_confidence;
                
                if (distance < bestHandDist){
                    handObjectIndex = i;
                    bestHandDist = distance;
                    bestDistConf = conf;
                }

                if (conf > SVM_HIGH_CONFIDENCE_THRESH) {
                    drawHand(handVisual, obj, conf);
                    ++handCount;

                    if (handObjectIndex == i) handObjectIndex = -1;
                }
            }
        }

        // Draw hands
        if (handObjectIndex != -1) {
            ++handCount;
            Object3D obj = objects[handObjectIndex];
            drawHand(handVisual, obj, bestDistConf);
        }

        if (handCount > 0) {
            cv::putText(handVisual,
                std::to_string(handCount) + " Hand" + (handCount == 1 ? "" : "s"),
                Point2i(10, 25), 0, 0.5, cv::Scalar(255, 255, 255));
        }


        if (cycleStartTime) {
            if (frame % FPS_FRAMES == 0) {
                currFps = (float)CLOCKS_PER_SEC  * (float)FPS_FRAMES / (clock() - cycleStartTime);
                cycleStartTime = clock();
            }

            if (frame > FPS_FRAMES) {
                std::stringstream fpsDisplay;
                double fpsDisp = currFps;
                fpsDisplay << "FPS: " << (fpsDisp < 10 ? "0" : "") << setprecision(3) << std::fixed << fpsDisp;
                cv::putText(handVisual, fpsDisplay.str(),
                    Point2i(handVisual.cols - 120, 25), 0, 0.5, cv::Scalar(255, 255, 255));

            }
        }
        else {
            cycleStartTime = clock();
        }

        if (camera->badInput()) {
            const int RECT_WID = 160, RECT_HI = 40;
            cv::Rect rect(handVisual.cols / 2 - RECT_WID / 2,
                handVisual.rows / 2 - RECT_HI / 2,
                RECT_WID, RECT_HI);
            cv::rectangle(handVisual, rect, cv::Scalar(0, 50, 255), -1);
            cv::putText(handVisual, "NO SIGNAL", 
                cv::Point(handVisual.cols / 2 - 65, handVisual.rows / 2 + 7),
                0, 0.8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }

        cv::namedWindow("OpenARK Hand Detection");
        cv::imshow("OpenARK Hand Detection", handVisual);

        /**** Start: Loop Break Condition ****/
        int c = cv::waitKey(1);

        // 27 is ESC
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }

        /**** End: Loop Break Condition ****/
        ++frame;
    }

    camera->destroyInstance();
    cv::destroyAllWindows();
    return 0;
}