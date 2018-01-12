#pragma once
#include "stdafx.h"

namespace ark {
    /**
    * Abstract class that defines the behavior of a RGB camera.
    */
    class RGBCamera
    {
    public:
        virtual ~RGBCamera() = default;
        /**
        * Updates the current frame on the RGB camera.
        * Should be overriden by a concerte implementation specific to the RGB camera
        */
        virtual void update();

        /**
        * Returns the current frame.
        * @return the current frame
        */
        cv::Mat getFrame() const;

    protected:
        /**
        * Camera handle.
        */
        cv::VideoCapture cap;

        /**
        * Current frame.
        */
        cv::Mat frame;
    };
}