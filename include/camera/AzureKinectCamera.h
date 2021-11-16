/* 
 * Azure Kinect Camera Class api for Hand & Avatr
 * 
 * Alex Yu ( alexyu99126@gmail.com ) 2019
 * Xiao Song ( xiaosx@berkeley.edu ) 2021
 * 
 * Change Log
 * 2021-10-25: optimize the implementation.
 */

#pragma once

#include <k4a/k4a.h> // openark api
#include <opencv2/core.hpp>
#include "camera/DepthCamera.h"

namespace ark {
    /**
    * Class defining the behavior of an Azure Kinect (K4A) Camera.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class AzureKinectCamera : public DepthCamera
    {
    public:

        /**
        * Public constructor initializing the Azure Kinect Camera.
        * @param device_id camera device id. 0 is default.
        * @param wide_fov_mode if true, starts Azure Kinect in wide FOV depth mode
        * @param use_1080p if true, records in 1080p rather than 720p
        * @param scale amount to scale down final image by
        */
        explicit AzureKinectCamera() noexcept;

        /**
        * Destructor for the Azure Kinect Camera.
        */
        ~AzureKinectCamera() override;

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
        const DetectionParams::Ptr & getDefaultParams() const override;

        /**
         * Returns true if an RGB image is available from this camera.
         * @return true if an RGB image is available from this camera.
         */
        bool hasRGBMap() const override;

        /** Preferred frame height */
        const int PREFERRED_FRAME_H = 480;

        /** Shared pointer to Azure Kinect camera instance */
        typedef std::shared_ptr<AzureKinectCamera> Ptr;

        /** Get the timestamp of the last image in nanoseconds */
        uint64_t getTimestamp() const;

        /** Get the basic calibration intrinsics
         *  @return (fx, cx, fy, cy) */
        cv::Vec4d getCalibIntrinsics() const;

    protected:
        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(cv::Mat & xyz_map, \
                    cv::Mat & rgb_map, \
                    cv::Mat & ir_map, \
                    cv::Mat & amp_map, \
                    cv::Mat & flag_map) override;

    private:
        k4a_device_t device;
        k4a_capture_t capture;
        k4a_calibration_t calibration;
        k4a_transformation_t transformation;
        k4a_device_configuration_t camera_config;
        k4a_image_t xy_lookup_table;
        // cx, cy, fx, fy
        cv::Vec4d intrinsic;
        uint16_t timestamp;
        int image_width, image_height;
        //int scale_width, scale_height;
        //double scale;
        // Timeout for capture
        const int32_t TIMEOUT_IN_MS = 1000;

        mutable bool defaultParamsSet = false;
        mutable DetectionParams::Ptr defaultParams;
    };
}