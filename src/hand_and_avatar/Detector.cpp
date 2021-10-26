#include "hand_and_avatar/Detector.h"
#include "hand_and_avatar/DetectionParams.h"

namespace ark {
    Detector::Detector(DetectionParams::Ptr params)
        : params(params ? params : DetectionParams::DEFAULT) {
        callback = std::bind(&Detector::callbackHelper, this, std::placeholders::_1);
    } 

    void Detector::update(const cv::Mat & image)
    {
        this->image = image;
        detect(this->image);
        lastCamera = nullptr;
        onSameFrame = false;
    }

    void Detector::update(DepthCamera & camera)
    {
        // stop if the camera is still on the same frame as before
        if (onSameFrame && lastCamera == &camera) return;
        this->image = camera.getXYZMap();
        detect(this->image);

        if (lastCamera != &camera) {
            if (lastCamera) lastCamera->removeUpdateCallback(lastUpdateCallbackID);
            lastUpdateCallbackID = camera.addUpdateCallback(callback);
        }

        lastCamera = &camera;
    }

    void Detector::setParams(const DetectionParams::Ptr params)
    {
        this->params = params;
    }

    void Detector::callbackHelper(DepthCamera & camera)
    {
        onSameFrame = false;
    }
}
