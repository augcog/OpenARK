#include <memory>
#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

#include "Core.h"
#include "UnityInterface.h"

#ifdef PMDSDK_ENABLED
    #include "PMDCamera.h"
#endif
#ifdef RSSDK_ENABLED
    #include "SR300Camera.h"
#endif

extern "C" {
    // static storage
    static ark::DepthCamera::Ptr camera = nullptr;
    static ark::HandDetector::Ptr hd;
    static ark::PlaneDetector::Ptr pd;
    static ark::DetectionParams::Ptr params;
    static const std::vector<ark::Hand::Ptr> * hands;
    static const std::vector<ark::FramePlane::Ptr> * planes;
    static std::vector<int> touches;

    static int lastTouchHand;

    static void init() {
#ifdef PMDSDK_ENABLED
        camera = std::make_shared<ark::PMDCamera>();
#endif
#ifdef RSSDK_ENABLED
        camera = std::make_shared<ark::SR300Camera>();
#endif
        params = camera->getDefaultParams();
        pd = std::make_shared<ark::PlaneDetector>(params);
        hd = std::make_shared<ark::HandDetector>(pd, params);
    }

    void update() {
        if (!camera || camera->badInput()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return;
        }
        pd->update(*camera);
        hd->update(*camera);
        planes = &pd->getPlanes();
        hands = &hd->getHands();
    }

    void beginCapture() {
        if (!camera) init();
        camera->beginCapture();
    }

    void endCapture() {
        camera->endCapture();
    }

    int numHands()
    {
        if (!hands) return 0;
        return (int) hands->size();
    }

    int numPlanes()
    {
        if (!planes) return 0;
        return (int) planes->size();
    }

    float handPos(int hand_id, int axis)
    {
        return hands->at(hand_id)->getPalmCenter()[axis];
    }

    float handAvgDepth(int hand_id)
    {
        return hands->at(hand_id)->getDepth();
    }

    float handDirection(int hand_id, int axis)
    {
        const ark::Point2f & pt = hands->at(hand_id)->getDominantDirection();
        if (axis == 0) return pt.x;
        else return pt.y;
    }

    float handWristPos(int hand_id, int index, int axis)
    {
        return hands->at(hand_id)->getWrist()[index][axis];
    }

    int handNumFingers(int hand_id)
    {
        return hd->getHands()[hand_id]->getNumFingers();
    }

    float handFingerPos(int hand_id, int index, int axis)
    {
        return hands->at(hand_id)->getFingers()[index][axis];
    }

    float handDefectPos(int hand_id, int index, int axis)
    {
        return hands->at(hand_id)->getDefects()[index][axis];
    }

    float planePos(int plane_id, int axis)
    {
        return planes->at(plane_id)->getCenter()[axis];
    }

    float planeEquation(int plane_id, int term) {
        return planes->at(plane_id)->equation[term];
    }

    float planePointDist(int plane_id, float x, float y, float z) {
        return planes->at(plane_id)->distanceToPoint(ark::Vec3f(x, y, z));
    }

    float planePointNorm(int plane_id, float x, float y, float z) {
        return planes->at(plane_id)->squaredDistanceToPoint(ark::Vec3f(x, y, z));
    }

    int computeTouches(int hand_id, int plane_id, float thresh) {
        const ark::FramePlane & plane = *planes->at(plane_id);
        lastTouchHand = hand_id;
        return hands->at(hand_id)->touchingPlane(plane, touches, thresh);
    }

    int numTouches() {
        return (int) touches.size();
    }

    int touchFingerIndex(int touch_id) {
        return touches[touch_id];
    }

    float touchPos(int touch_id, int axis) {
        return hands->at(lastTouchHand)->getFingers()[touches[touch_id]][axis];
    }

    void handUseSVM(bool value)
    {
        params->handUseSVM = value;
    }

    void handRequireEdgeConnected(bool value)
    {
        params->handRequireEdgeConnected = value;
    }
}