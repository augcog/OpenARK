#pragma once
// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>

// OpenARK Libraries
#include "camera/CameraSetup.h"

namespace ark {
    /**
    * Class defining the behavior of a generic Intel RealSense Camera using RSSDK2.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class D435Camera : public CameraSetup
    {
    public:

        /**
        * Public constructor initializing the RealSense Camera.
        * @param use_rgb_stream if true, uses the RGB stream and disable the IR stream (which is on by default)
        *                       This results in a smaller field of view and has an appreciable performance cost.
        */
        explicit D435Camera();

        /**
        * Destructor for the RealSense Camera.
        */
        ~D435Camera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /**
         * Get image size
         */
        cv::Size getImageSize() const;

        /** 
         * Sets the external hardware sync ans starts the camera
         */
        void start() override;

        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(MultiCameraFrame & frame) override;

    protected:

        /** Converts an D435 raw depth image to an ordered point cloud based on the current camera's intrinsics */
        void project(const rs2::frame & depth_frame, cv::Mat & xyz_map);

        std::shared_ptr<rs2::pipeline> pipe;
        rs2::config config;
        rs2::depth_sensor* depth_sensor;
        rs2::device device;
        rs2_intrinsics depthIntrinsics;

        double scale;
        int width, height;
        bool badInputFlag;
    };
}
