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

const char* SVM_MODEL_PATH = "svm/";
const double SVM_PREDICT_BOUNDARY = 0.5;

#ifdef PROFILING
const char* profCategories[] = { "Update", "Denoise", "Clustering", "Hand Identification", "Other" };
#endif

int main() {
    DepthCamera * camera = nullptr;

#ifdef PMDSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "pmd")) {
        camera = new PMDCamera();
    }
    else {
        return 0;
    }
#endif
#ifdef RSSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "sr300")) {
        camera = new SR300Camera();
    }
    else {
        return 0;
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

    StreamingAverager handAverager = StreamingAverager(4, 0.1), paleeteAverager = StreamingAverager(6, 0.05);

    // Initialize SVM Hand Classifier
    classifier::HandClassifier & handClassifier = classifier::SVMHandClassifier(SVM_MODEL_PATH);

#ifdef PROFILING
    // -- Profiling code --

    // cycle count
    int profCycles = 0;

    // set starting time
    clock_t lastTime = clock(), delta, totalTime = 0, cycleStartTime = 0;

    // names of profiling categories, change/add as need
    clock_t profTimes[sizeof profCategories / sizeof profCategories[0]];

    // set all times to 0 initially
    memset(profTimes, 0, sizeof profTimes);
#endif

    // store FPS information
    const int FPS_FRAMES = 5;
    float currFps = 0;

    while (true)
    {
        camera->update();

#ifdef PROFILING
        ++profCycles;
        delta = clock() - lastTime; profTimes[0] += delta; totalTime += delta; lastTime = clock();
#endif

        // Do noise removal
        camera->removeNoise();

#ifdef PROFILING
        delta = clock() - lastTime; profTimes[1] += delta; totalTime += delta; lastTime = clock();
#endif

        // Bad input, wait a little before trying again
        if (camera->badInput) {
            waitKey(10);
            #ifdef PROFILING
                lastTime = clock();
            #endif
            continue;
        }

        // Classifying objects in the scene
        camera->computeClusters(0.667, 750, 15);
        auto clusters = camera->getClusters();
        std::vector<Object3D> objects;

        int handObjectIndex = -1, handCount = 0, planeObjectIndex = -1;
        float closestHandDist = FLT_MAX;

#ifdef PROFILING
        delta = clock() - lastTime; profTimes[2] += delta; totalTime += delta; lastTime = clock();
#endif

        // visualization of hand objects
        cv::Mat handVisual = cv::Mat::zeros(camera->getHeight() * 2, camera->getWidth() * 2, CV_8UC3);

        for (int i = 0; i < clusters.size(); ++i) {
            objects.emplace_back(clusters[i]);

            Object3D obj = objects.back();

            if (obj.hasHand) {
                std::vector<double> feats = classifier::features::extractHandFeatures(obj, clusters[i]);

                // get SVM prediction (confidence value in [0, 1])
                double predict = handClassifier.classify(feats);

                if (predict > SVM_PREDICT_BOUNDARY) {
                    ++handCount;
                    
                    // Draw this hand
                    handVisual = Visualizer::visualizeHand(handVisual, obj.getHand());
                    cv::polylines(handVisual, obj.getComplexContour(), true, cv::Scalar(100, 255, 100), 1);
                    
                    if (obj.getHand().centroid_xyz[2] < closestHandDist) {
                        handObjectIndex = i;
                        closestHandDist = obj.getHand().centroid_xyz[2];
                    }
                }
                else {
                    // Candidate considered but rejected (draw outline in gray)
                    cv::polylines(handVisual, obj.getComplexContour(), true, cv::Scalar(100, 100, 200));
                }

                // display confidence value at center of object
                std::stringstream predictionDisp;
                predictionDisp << std::setprecision(3) << std::fixed << predict;

                cv::Point dispPt = cv::Point(obj.getHand().centroid_ij.x - 60, obj.getHand().centroid_ij.y);

                cv::putText(handVisual, predictionDisp.str(),
                    dispPt, 0, 2, cv::Scalar(100, 255, 100));

            }

            if (obj.hasPlane) {
                planeObjectIndex = i;
            }
        }

        if (handObjectIndex != -1) {
            cv::polylines(handVisual, objects[handObjectIndex].getComplexContour(),
                true, cv::Scalar(0, 255, 0), 2);
        }

#ifdef PROFILING
        delta = clock() - lastTime; profTimes[3] += delta; totalTime += delta; lastTime = clock();
#endif

        // Show visualizations
        cv::Mat handScaled;
        cv::resize(handVisual, handScaled, cv::Size(camera->getWidth(), camera->getHeight()));

        // cv::namedWindow("Nearest Hand", cv::WINDOW_AUTOSIZE);
        if (handObjectIndex != -1) {
            // cv::imshow("Nearest Hand", clusters[handObjectIndex]);
            cv::putText(handScaled,
                std::to_string(handCount) + " Hand" + (handCount == 1 ? "" : "s"),
                cv::Point(10, 25), 0, 0.5, cv::Scalar(255, 255, 255));
        }
        // else {
            // cv::imshow("Nearest Hand", handScaled);
            // cv::putText(handScaled, "No Hands", cv::Point(10, 25), 0, 0.5, cv::Scalar(255,255,255));
        // }

        if (cycleStartTime) {
            if (frame % FPS_FRAMES == 0) {
                currFps = (float)CLOCKS_PER_SEC  * (float)FPS_FRAMES / (clock() - cycleStartTime);
                cycleStartTime = clock();
            }

            if (frame > FPS_FRAMES) {
                std::stringstream fpsDisplay;
                fpsDisplay << "FPS: " << (currFps < 10 ? "0" : "") << setprecision(3) << std::fixed << currFps;
                cv::putText(handScaled, fpsDisplay.str(),
                    cv::Point(handScaled.cols - 120, 25), 0, 0.5, cv::Scalar(255, 255, 255));
            }
        }
        else {
            cycleStartTime = clock();
        }

        cv::namedWindow("OpenARK Hand Detection");
        cv::imshow("OpenARK Hand Detection", handScaled);

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

#ifdef PROFILING
        delta = clock() - lastTime; profTimes[4] += delta; totalTime += delta; lastTime += delta;
#endif

        /**** Start: Write Frames to File ****/
        //std::string filename = "img" + std::to_string(frame) + ".yml";
        //pmd->writeImage(filename);
        //std::cout << filename << std::endl;
        /**** End: Write Frames to File ****/

        /**** Start: Loop Break Condition ****/
        auto c = waitKey(1);
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }

        /**** End: Loop Break Condition ****/
        ++frame;

#ifdef PROFILING
        // get profiling report
        if (c == 't' || c == 'T') {
            float profDelay = (float)totalTime / profCycles / CLOCKS_PER_SEC;

            printf("--PROFILING REPORT--\nTotal: %d\nDelay: %f s\nFPS: %f\n\n",
                totalTime, profDelay, 1/profDelay);
            for (int i = 0; i < sizeof profTimes / sizeof profTimes[0]; ++i) {
                printf("%s: %d (%f%%)\n", profCategories[i], profTimes[i], (float)profTimes[i] / totalTime * 100);
            }
            printf("--END OF REPORT--\n\n");
        }
        // reset time
        else if (c == 'r' || c == 'R') {
            totalTime = profCycles = 0;
            // clear all times
            memset(profTimes, 0, sizeof profTimes);
            printf("Profiling times reset.\n");
        }
        lastTime = clock();
#endif
    }


    camera->destroyInstance();
    destroyAllWindows();
    return 0;
}