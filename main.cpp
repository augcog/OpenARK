#include "stdafx.h"
#include "string.h"
// OpenARK Libraries
#include "Version.h"
#include "ProtobufHand.pb.h"
#include "ProtobufImage.pb.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Core.h"
#include "Visualizer.h"
#include "NetworkStereoCamera.h"
using namespace ark;

static inline void setPoint(protob::Point * out, const Vec3f & xyz, const Point & ij) {
    protob::PointXYZ * xyz_pb = out->mutable_pointxyz();
    protob::PointIJ * ij_pb = out->mutable_pointij();
    xyz_pb->set_x(xyz[0]);
    xyz_pb->set_y(xyz[1]);
    xyz_pb->set_z(xyz[2]);
    ij_pb->set_i(ij.x);
    ij_pb->set_j(ij.y);
}

static void initGUI(cv::Mat * roi) {
    cv::namedWindow("Depth Map");
    cv::namedWindow("Demo Output");

    // click on depth map viewer to see depth numbers
    cv::setMouseCallback("Depth Map", [](int event, int x, int y, int flags, void* userdata) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            cv::Mat m = *((cv::Mat *)userdata);
            std::cout << x << "," << y << ": " << m.at<cv::Vec3f>(y / 1.5, x / 1.5) << "\n";
        }
    }, roi);
}

int main() {
    // seed the rng
    srand(time(NULL));

    auto calib = StereoCalibration::create("../calib_sample.yaml");
    NetworkStereoCamera cam(calib, 23333);

    // initialize parameters
    DetectionParams::Ptr params = cam.getDefaultParams(); // default parameters

    // initialize detectors
    PlaneDetector::Ptr planeDetector = std::make_shared<PlaneDetector>();
    HandDetector::Ptr handDetector = std::make_shared<HandDetector>(planeDetector);

    // store frame & FPS information
    const int FPS_CYCLE_FRAMES = 8; // number of frames to average FPS over (FPS 'cycle' length)
    auto cycleStartTime = std::chrono::high_resolution_clock::now(); // start time of current cycle
    float currFPS; // current FPS

    // option flags
    bool useSVM = false, useEdgeConn = true, showArea = false;

    cv::Mat roi;
    initGUI(&roi);

    // last frame timestamp
    size_t last_time = 0;

    // begin capture
    cam.beginCapture();

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // main demo loop
	while (true)
	{
        while (!cam.frameReady()) {
            // wait until camera ready
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto frame = cam.getLatestFrame();
        if (frame.timestamp <= last_time) continue;
        last_time = frame.timestamp;
        cv::Mat xyzMap = frame.xyzMap, leftImage = frame.leftImage;

		/**** Start: Hand/plane detection ****/
		params->handUseSVM = useSVM;
		params->handRequireEdgeConnected = useEdgeConn;

		std::vector<Hand::Ptr> hands;
		std::vector<FramePlane::Ptr> planes;

		planeDetector->setParams(params);
		handDetector->setParams(params);

        int rows = xyzMap.rows;
        int cols = xyzMap.cols;
        cv::Rect rect(105, 10, cols - 110, rows - 20);
        xyzMap(rect).copyTo(roi);

        float Fx_ = 1.5, Fy_ = 1.5;
        planeDetector->update(roi);
        planes = planeDetector->getPlanes();

        handDetector->update(roi);
        hands = handDetector->getHands();

        if (hands.size() > 0) {
            std::cout << "num fingers on hand: " << hands[0]->getFingers().size() << std::endl;
        }

        // send hands to device
        protob::Hands hands_pb;
        for (const auto & hand : hands) {
            protob::Hand  * hand_pb = hands_pb.add_hands();
            setPoint(hand_pb->mutable_palmcenter(), hand->getCenter(), hand->getCenterIJ());
            setPoint(hand_pb->add_wrist(), hand->getWrist()[0], hand->getWristIJ()[0]);
            setPoint(hand_pb->add_wrist(), hand->getWrist()[1], hand->getWristIJ()[1]);
            for (int i = 0; i < hand->getNumFingers(); ++i) {
                setPoint(hand_pb->add_fingers(), hand->getFingers()[i], hand->getFingersIJ()[i]);
            }
        }
        std::string hands_str = hands_pb.SerializeAsString();
        cam.sendToClient(hands_str, frame.clientID, true);
        /**** End: Hand/plane detection ****/

        /**** Start: Visualization ****/
        // construct visualizations
        cv::Mat handVisual;

        leftImage(rect).copyTo(handVisual);
        cvtColor(handVisual, handVisual, cv::COLOR_GRAY2RGB);

        const cv::Scalar WHITE(255, 255, 255);

        // draw hands
        for (Hand::Ptr hand : hands) {
            double dispVal;
            if (showArea) {
                dispVal = hand->getSurfArea();
            }
            else if (useSVM) {
                dispVal = hand->getSVMConfidence();
            }
            else {
                dispVal = FLT_MAX;
            }

            //Draw hand on original image
            Visualizer::visualizeHand(handVisual, handVisual, hand.get(), dispVal/*, &planes*/);
        }

        if (hands.size() > 0) {
            // show "N Hands" on top left1
            cv::putText(handVisual, std::to_string(hands.size()) +
                util::pluralize(" Hand", hands.size()),
                Point2i(10, 25), 0, 0.5, WHITE);
        }

        // update FPS
        if (frame.timestamp % FPS_CYCLE_FRAMES == 0) {
            auto now = std::chrono::high_resolution_clock::now();
            currFPS = (float)FPS_CYCLE_FRAMES * 1000.0f / std::chrono::duration_cast<std::chrono::milliseconds>(now - cycleStartTime).count();
            cycleStartTime = now;
        }
        if (frame.timestamp > FPS_CYCLE_FRAMES) {
            // show FPS on top right
            std::stringstream fpsDisplay;
            static char chr[32];
            sprintf(chr, "FPS: %02.3f", currFPS);
            cv::putText(handVisual, chr, Point2i(handVisual.cols - 120, 25), 0, 0.5, WHITE);
        }

        // show visualizations
        if (!xyzMap.empty()) {
            cv::Mat vis;

            Visualizer::visualizeXYZMap(roi, vis, 5.5f);
            cv::imshow("Depth Map", vis);
        }

        cv::imshow("Demo Output", handVisual);

        /**** End: Visualization  ****/
        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    return 0;
}
