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

using namespace cv;

static inline void drawHand(cv::Mat & image, Object3D & obj, float confidence) {
    if (obj.getContour().size() > 2) {
        cv::polylines(image, obj.getContour(), true, cv::Scalar(0, 255, 0), 1);
    }

    if (obj.hasHand) {
        image = Visualizer::visualizeHand(image, obj.getHand());
    }

    std::stringstream disp;
    disp << std::setprecision(3) << std::fixed << confidence;

    cv::Point center = obj.getCenterIj();
    cv::Point dispPt = cv::Point(center.x - (int)disp.str().size() * 8, center.y);
    cv::putText(image, disp.str(), dispPt, 0, 0.8, cv::Scalar(255, 255, 255), 1);
}

int main() {

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

    //RGBCamera *cam = new Webcam(1);
    int frame = 0;

    //Calibration::XYZToUnity(*pmd, 4, 4, 3);

    FileStorage fs;
    fs.open("RT_Transform.txt", FileStorage::READ);

    Mat r, t;
    fs["R"] >> r;
    fs["T"] >> t;

    fs.release();
    UDPSender u = UDPSender();

    StreamingAverager handAverager = StreamingAverager(4, 0.15), paleeteAverager = StreamingAverager(6, 0.05);

    clock_t cycleStartTime = 0;

    // store FPS information
    const int FPS_FRAMES = 5;
    float currFps = 0;

    while (true)
    {
        camera->nextFrame();

        #ifdef DEMO
            if (camera->getXYZMap().rows > 0) {
                cv::imshow("OpenARK Depth Image", camera->getXYZMap());
                if (camera->hasRGBImage()) {
                    cv::imshow("OpenARK Color Image", camera->getRGBImage());
                }
                //if (camera->hasIRImage() && camera->getIRImage().rows > 0) {
                //    cv::Mat colorMat;
                //    camera->getIRImage().convertTo(colorMat, CV_8U);
                //    cv::imshow("OpenARK Infrared Image", colorMat);
                //}
            }
        #endif

        // Classifying objects in the scene
        std::vector<Object3D> objects = camera->queryObjects();

        // visualization of hand objects
        cv::Mat handVisual;

#ifdef DEMO
        if (camera->hasIRImage() && camera->getIRImage().rows > 0) {
            camera->getIRImage().convertTo(handVisual, CV_8UC1);
            cv::cvtColor(handVisual, handVisual, cv::COLOR_GRAY2BGR, 3);
            handVisual /= 2;
        }
        else {
#endif
            handVisual = cv::Mat::zeros(camera->getHeight(), camera->getWidth(), CV_8UC3);
#ifdef DEMO
        }
#endif

        int handObjectIndex = -1, handCount = 0, planeObjectIndex = -1;

        // object is "certainly" if SVM confidence is above this
        static const double SVM_HIGH_CONFIDENCE_THRESH = 0.59;

        float bestHandDist = FLT_MAX, bestDistConf = 0;

        for (int i = 0; i < objects.size(); ++i) {  
            Object3D & obj = objects[i];

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

            if (obj.hasPlane) {
                planeObjectIndex = i;
            }
        }

        // Show visualizations
        if (handObjectIndex != -1) {
            ++handCount;
            Object3D obj = objects[handObjectIndex];
            drawHand(handVisual, obj, bestDistConf);
        }

        if (handCount > 0) {
            cv::putText(handVisual,
                std::to_string(handCount) + " Hand" + (handCount == 1 ? "" : "s"),
                cv::Point(10, 25), 0, 0.5, cv::Scalar(255, 255, 255));
        }


        if (cycleStartTime) {
            if (frame % FPS_FRAMES == 0) {
                currFps = (float)CLOCKS_PER_SEC  * (float)FPS_FRAMES / (clock() - cycleStartTime);
                cycleStartTime = clock();
            }

            if (frame > FPS_FRAMES) {
                std::stringstream fpsDisplay;
                fpsDisplay << "FPS: " << (currFps < 10 ? "0" : "") << setprecision(3) << std::fixed << currFps;
                cv::putText(handVisual, fpsDisplay.str(),
                    cv::Point(handVisual.cols - 120, 25), 0, 0.5, cv::Scalar(255, 255, 255));

            }
        }
        else {
            cycleStartTime = clock();
        }

        cv::namedWindow("OpenARK Hand Detection");
        cv::imshow("OpenARK Hand Detection", handVisual);

        // Interpret the relationship between the objects
        bool clicked = false, paletteFound = false;

        Object3D handObject, planeObject;
        Point paletteCenter(-1. - 1);
        Mat mask = Mat::zeros(camera->getXYZMap().rows, camera->getXYZMap().cols, CV_8UC1);

        if (planeObjectIndex != -1 && handObjectIndex != -1) {
            planeObject = objects[planeObjectIndex];
            handObject = objects[handObjectIndex];

            clicked = handObject.getHand().touchObject(planeObject.getPlane().getPlaneEquation(), planeObject.getPlane().R_SQUARED_DISTANCE_THRESHOLD * 5);
            //auto scene = Visualizer::visualizePlaneRegression(camera->getXYZMap(), planeObject.getPlane().getPlaneEquation(), planeObject.getPlane().R_SQUARED_DISTANCE_THRESHOLD, clicked);
            //scene = Visualizer::visualizeHand(scene, handObject.getHand());
            if (planeObject.leftEdgeConnected) {
                Visualizer::visualizePlanePoints(mask, planeObject.getPlane().getPlaneIndicies());
                auto m = moments(mask, false);
                paletteCenter = Point(m.m10 / m.m00, m.m01 / m.m00);
                //circle(scene, paletteCenter, 2, Scalar(0, 0, 255), 2);
                paletteFound = true;
            }
            //namedWindow("Results", CV_WINDOW_AUTOSIZE);
            //imshow("Results", scene);
        }
        else if (handObjectIndex != -1) {
            handObject = objects[handObjectIndex];
            //cv::imshow("Results", Visualizer::visualizeHand(pmd->getXYZMap(), handObject.getHand()));
        }
        else if (planeObjectIndex != -1) {
            planeObject = objects[planeObjectIndex];
            //auto scene = Visualizer::visualizePlaneRegression(camera->getXYZMap(), planeObject.getPlane().getPlaneEquation(), planeObject.getPlane().R_SQUARED_DISTANCE_THRESHOLD, clicked);
            if (planeObject.leftEdgeConnected) {
                Visualizer::visualizePlanePoints(mask, planeObject.getPlane().getPlaneIndicies());
                auto m = moments(mask, false);
                paletteCenter = Point(m.m10 / m.m00, m.m01 / m.m00);
                //circle(scene, paletteCenter, 2, Scalar(0, 0, 255), 2);
                paletteFound = true;
            }
            //namedWindow("Results", CV_WINDOW_AUTOSIZE);
            //imshow("Results", scene);
        }

        // Organize the data and send to game engine
        std::string handX = "-", handY = "-", handZ = "-";
        std::string paletteX = "-", paletteY = "-", paletteZ = "-";
        std::string clickStatus = "2";
        std::string num_fingers = "0";
        if (handObjectIndex != -1) {
            auto handPos = handAverager.addDataPoint(objects[handObjectIndex].getHand().fingers_xyz[0]);
            //float hand_pt[3] = { objects[handObjectIndex].getHand().pointer_finger_xyz[0], objects[handObjectIndex].getHand().pointer_finger_xyz[1], objects[handObjectIndex].getHand().pointer_finger_xyz[2]};
            double hand_pt[3] = { handPos[0], handPos[1], handPos[2] };
            auto hand_mat = Mat(3, 1, CV_32FC1, &hand_pt);
            //hand_mat = r*hand_mat + t;
            handX = std::to_string(hand_mat.at<float>(0, 0));
            handY = std::to_string(hand_mat.at<float>(1, 0));
            handZ = std::to_string(hand_mat.at<float>(2, 0));
            num_fingers = std::to_string(objects[handObjectIndex].getHand().fingers_xyz.size());
        }
        else {
            handAverager.addEmptyPoint();
        }
        if (paletteFound) {
            auto pt = paleeteAverager.addDataPoint(camera->getXYZMap().at<Vec3f>(paletteCenter.y, paletteCenter.x));
            float palette_pt[3] = { pt[0], pt[1], pt[2] };
            auto palette_mat = Mat(3, 1, CV_32FC1, &palette_pt);
            //palette_mat = r*palette_mat + t;
            paletteX = std::to_string(palette_mat.at<float>(0, 0));
            paletteY = std::to_string(palette_mat.at<float>(1, 0));
            paletteZ = std::to_string(palette_mat.at<float>(2, 0));
        }
        else {
            paleeteAverager.addEmptyPoint();
        }
        if (clicked) {
            clickStatus = "1";
        }

        std::string tempS = "";
        tempS = handX + "%" + handY + "%" + handZ + "%" + paletteX + "%" + paletteY + "%" + paletteZ + "%" + clickStatus + "%" + num_fingers;
        u.send(tempS);

        /**** Start: Write Frames to File ****/
        //std::string filename = "img" + std::to_string(frame) + ".yml";
        //pmd->writeImage(filename);
        //std::cout << filename << std::endl;
        /**** End: Write Frames to File ****/

        /**** Start: Loop Break Condition ****/
        int c = waitKey(1);
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }

        /**** End: Loop Break Condition ****/
        ++frame;
    }

    camera->destroyInstance();
    destroyAllWindows();
    return 0;
}