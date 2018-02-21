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
#include "FramePlane.h"
#include "Calibration.h"
#include "Util.h"
#include "UDPSender.h"
#include "FrameObject.h"
#include "StreamingAverager.h"

#include "HandClassifier.h"

using namespace ark;

int main() {
    printf("Welcome to OpenARK v %s Demo\n\n", ark::VERSION);
    printf("CONTROLS:\nQ or ESC to quit, P to show/hide planes, H to show/hide hands, SPACE to play/pause\n\n");
    printf("VIEWER BACKGROUNDS:\n1 = IR Image, 2 = Depth Image, 3 = Normal Map, 0 = None\n");
    boost::shared_ptr<DepthCamera> camera;

#ifdef PMDSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "pmd")) {
        camera = boost::make_shared<PMDCamera>();
    }
    else {
        return -1;
    }
#endif
#ifdef RSSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "sr300")) {
        camera = boost::make_shared<SR300Camera>();
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

    unsigned long long minFrameTime = 1000 / 120; // 1000 / FPS CAP

    // image layer flags
    bool handLayer = true, planeLayer = false;

    // background style: 0=none, 1=ir, 2=depth, 3=normal
    int backgroundStyle = 1;

    // if false, pauses picture
    bool playing = true;

    // turn on the camera
    camera->beginCapture(120, false);

    // main demo loop
    while (true)
    {
        // get latest images from the camera
        cv::Mat xyzMap = camera->getXYZMap();

        // query objects in the current frame
        ObjectParams params; // default parameters

        std::vector<ark::HandPtr> hands;
        std::vector<ark::FramePlanePtr> planes;
        
        if (handLayer || planeLayer) {
            planes = camera->getFramePlanes(&params);

            if (handLayer) {
                hands = camera->getFrameHands(&params);
            }
        }

        // construct visualizations
        cv::Mat handVisual;

        // background of visualization
        if (backgroundStyle == 1 && camera->hasIRMap()) {
            // IR background
            cv::cvtColor(camera->getIRMap(), handVisual, cv::COLOR_GRAY2BGR, 3);
        }

        else if (backgroundStyle == 2) {
            // depth map background
            Visualizer::visualizeXYZMap(xyzMap, handVisual);
        }

        else if (backgroundStyle == 3) {
            // normal map background
            cv::Mat normalMap = camera->getNormalMap();
            Visualizer::visualizeNormalMap(normalMap, handVisual, params.normalResolution);
        }

        else {
            handVisual = cv::Mat::zeros(camera->getImageSize(), CV_8UC3);
        }

        const cv::Scalar WHITE(255, 255, 255);

        if (planeLayer) {
            // color all points on the screen close to a plane
            cv::Vec3s color;

            for (uint i = 0; i < planes.size(); ++i) {
                color = util::paletteColor(i);

                for (int row = 0; row < xyzMap.rows; ++row) {
                    const Vec3f * ptr = xyzMap.ptr<Vec3f>(row);
                    Vec3b * outPtr = handVisual.ptr<Vec3b>(row);

                    for (int col = 0; col < xyzMap.cols; ++col) {
                        const Vec3f & xyz = ptr[col];
                        if (xyz[2] == 0) continue;

                        float norm = util::pointPlaneNorm(xyz, planes[i]->equation);

                        float fact = std::max(0.0, 0.5f - norm / params.handPlaneMinNorm / 8.0f);
                        if (fact == 0.0f) continue;

                        outPtr[col] += (color - (cv::Vec3s)outPtr[col]) * fact;
                    }
                }

                // draw normal vector
                Point2i drawPt = planes[i]->getCenterIJ();
                cv::Vec3f normal = planes[i]->getNormalVector();
                Point2i arrowPt(drawPt.x + normal[0] * 100, drawPt.y - normal[1] * 100);
                cv::arrowedLine(handVisual, drawPt, arrowPt, WHITE, 4, cv::LINE_AA, 0, 0.2);
            }
        }

        // draw hands
        if (handLayer) {
            for (ark::HandPtr hand : hands) {
                Visualizer::visualizeHand(handVisual, handVisual, hand.get(),
                                          hand->getSVMConfidence(), &planes);
            }
        }

        if (handLayer && hands.size() > 0) {
            // show "N Hands" on top left
            cv::putText(handVisual, std::to_string(hands.size()) +
                util::pluralize(" Hand", hands.size()),
                Point2i(10, 25), 0, 0.5, WHITE);
        }

        if (planeLayer && planes.size() > 0) {
            // show "N Planes" on top left
            cv::putText(handVisual, std::to_string(planes.size()) +
                util::pluralize(" Plane", planes.size()),
                Point2i(10, 55), 0, 0.5, WHITE);
        }

        // update FPS
        if (cycleStartTime) {
            if (frame % FPS_FRAMES == 0) {
                currFps = (float)CLOCKS_PER_SEC  * (float)FPS_FRAMES / (clock() - cycleStartTime);
                cycleStartTime = clock();
            }

            if (frame > FPS_FRAMES && !camera->badInput()) {
                // show FPS on top right
                std::stringstream fpsDisplay;
                double fpsDisp = currFps;
                static char chr[32];
                sprintf(chr, "FPS: %02.3lf", fpsDisp);
                Point2i pos(handVisual.cols - 120, 25);
                cv::putText(handVisual, chr, pos, 0, 0.5, WHITE);
            }
        }
        else {
            cycleStartTime = clock();
        }

        int wait = 1;

        // show "NO SIGNAL" on each visual in case of bad input
        if (camera->badInput() || !playing) {
            // wait 50 ms, or pause
            wait = !playing ? 0 : 50;

            const int RECT_WID = (!playing ? 120 : 160), RECT_HI = 40;
            cv::Rect rect(handVisual.cols / 2 - RECT_WID / 2,
                handVisual.rows / 2 - RECT_HI / 2,
                RECT_WID, RECT_HI);

            const cv::Scalar RECT_COLOR = cv::Scalar(0, (!playing ? 160 : 50), 255);
            const std::string NO_SIGNAL_STR = (!playing ? "PAUSED" : "NO SIGNAL");
            const cv::Point STR_POS(handVisual.cols / 2 - (!playing ? 50 : 65), handVisual.rows / 2 + 7);

            // show on each visual
            cv::rectangle(handVisual, rect, RECT_COLOR, -1);
            cv::putText(handVisual, NO_SIGNAL_STR, STR_POS, 0, 0.8, WHITE, 1, cv::LINE_AA);
            if (xyzMap.rows) {
                cv::rectangle(xyzMap, rect, RECT_COLOR / 255.0, -1);
                cv::putText(xyzMap, NO_SIGNAL_STR, STR_POS, 0, 0.8, cv::Scalar(1.0f, 1.0f, 1.0f), 1, cv::LINE_AA);
            }
        }

        // show visualizations
        cv::imshow(camera->getModelName() + " Depth Map", xyzMap);
        cv::imshow("Demo Output - OpenARK v" + std::string(ark::VERSION), handVisual);

        // END HAND DETECTION

        /**** Start: Loop Break Condition ****/
        int c = cv::waitKey(wait);

        // 27 is ESC
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }

        else if (c == 'p' || c == 'P') {
            planeLayer = !planeLayer;
        }

        else if (c == 'h' || c == 'H') {
            handLayer = !handLayer;
        }

        else if (c >= '0' && c <= '3') {
            backgroundStyle = c - '0';
        }

        else if (c == ' ') {
            // paused, any key to go to next frame (space to play)
            playing = !playing;
        }

        /**** End: Loop Break Condition ****/
        ++frame;
    }

    camera->endCapture();

    cv::destroyAllWindows();
    return 0;
}