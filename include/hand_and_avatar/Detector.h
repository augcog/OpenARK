#pragma once
#include "camera/DepthCamera.h"
#include <util/FrameObject.h>

namespace ark {
    /** Abstract object detector class. 
     * @see HandDetector
     * @see PlaneDetector
     */
    class Detector {
    public:
        /** Construct a new detector instance
          * @param params detection parameters. If not specified, uses default parameter values.
          */
        Detector(DetectionParams::Ptr params = nullptr);

        /**
         * Update this detector with the given image.
         * @param image the xyz map
         * @param params detection parameters (if not provided, uses default parameters)
         * @see DetectionParams
         */
        void update(const cv::Mat & image);
        
        /**
         * Update this detector with a frame taken from the given depth camera.
         * @param camera the depth camera
         * @param params detection parameters (if not provided, uses default parameters)
         * @see DetectionParams
         */
        void update(DepthCamera & camera);

        /** Change this detector's object detection parameters. */
        void setParams(const DetectionParams::Ptr params);

        /** Shared pointer to Detector instance */
        typedef std::shared_ptr<Detector> Ptr;

    protected:
        /** Primary function for object detection, called after each update. Must override in child classes. */
        virtual void detect(cv::Mat & image) = 0;

        /** Pointer to this detector's object detection parameters */
        DetectionParams::Ptr params; 

    private:
        /** Stores the XYZ map for the current frame */
        cv::Mat image;

        /** Pointer to last-used depth camera. null if last frame is from */
        DepthCamera * lastCamera = nullptr;

        /** True if camera frame has not updated since last update call (and so we don't need to recompute anything) */
        bool onSameFrame = true;

        /** ID of last callback function */
        int lastUpdateCallbackID = -1;

        /** callback function for responding to depth cameras updates */
        void callbackHelper(DepthCamera & camera);
        
        /** instance of callback function */
        std::function<void(DepthCamera &camera)> callback;
    };
}