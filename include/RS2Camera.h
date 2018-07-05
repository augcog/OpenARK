#pragma once
// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>

// OpenARK Libraries
#include "DepthCamera.h"

namespace ark {
    /**
    * Class defining the behavior of a generic Intel RealSense Camera using RSSDK2.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class RS2Camera : public DepthCamera
    {
    public:

        /**
        * Public constructor initializing the RealSense Camera.
        * @param use_rgb_stream if true, uses the RGB stream and disable the IR stream (which is on by default)
        *                       This results in a smaller field of view and has an appreciable performance cost.
        */
        explicit RS2Camera(bool use_rgb_stream = false);

        /**
        * Destructor for the RealSense Camera.
        */
        ~RS2Camera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /** 
         * Returns the width of the SR300 camera frame 
         */
        int getWidth() const override;

        /** 
         * Returns the height of the SR300 camera frame 
         */
        int getHeight() const override;

        /**
         * Returns default detection parameters for this depth camera class
         */
        DetectionParams::Ptr getDefaultParams() const override;

        /**
         * Returns true if an RGB image is available from this camera.
         * @return true if an RGB image is available from this camera.
         */
        bool hasRGBMap() const override;

        /**
         * Returns true if an infrared (IR) image is available from this camera.
         * @return true if an infrared (IR) image is available from this camera.
         */
        bool hasIRMap() const override;


        /** Preferred frame height */
        const int PREFERRED_FRAME_H = 480;

        /** Shared pointer to SR300 camera instance */
        typedef std::shared_ptr<RS2Camera> Ptr;

    protected:
        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                            cv::Mat & amp_map, cv::Mat & flag_map) override;

        /**
         * Initialize the camera, opening channels and resetting to initial configurations
         */
        void initCamera();

        /** Converts an RS2 raw depth image to an ordered point cloud based on the current camera's intrinsics */
        void project(const rs2::frame & depth_frame, const rs2::frame & rgb_frame, cv::Mat & xyz_map, cv::Mat & rgb_map);

        /** Query RealSense camera intrinsics */
        void query_intrinsics();

        // internal storage
        std::shared_ptr<rs2::pipeline> pipe;
        rs2::align align;
        rs2::config config;

        // pointer to depth sensor intrinsics (RealSense C API: rs_intrinsics)
        void * depthIntrinsics = nullptr;
        // pointer to RGB/IR sensor intrinsics (RealSense C API: rs_intrinsics)
        void * rgbIntrinsics = nullptr;
        // pointer to depth-to-RGB extrinsics (RealSense C API: rs_extrinsics)
        void * d2rExtrinsics = nullptr;
        // pointer to RGB-to-depth extrinsics (RealSense C API: rs_extrinsics)
        void * r2dExtrinsics = nullptr;

        double scale;
        int width, height;
        bool useRGBStream;

        mutable bool defaultParamsSet = false;
        mutable DetectionParams::Ptr defaultParams;
    };
}
