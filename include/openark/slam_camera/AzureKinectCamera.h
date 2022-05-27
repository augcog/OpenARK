/*
 * Azure Kinect Camera Class Header File
 *
 * Xiao Song ( xiaosx@berkeley.edu )
 * 
 */

#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp> // cv::Size, cv::Vec4d
#include <k4a/k4a.h> // k4a_device_t

namespace ark
{

    // Design: currently we only support AzureKinect camera. 
    // Thus there's no need to derive from base class.
    class AzureKinectCamera
    {
    public:
        typedef union
        {
            /** XYZ or array representation of vector. */
            struct _xyz
            {
                float x; /**< X component of a vector. */
                float y; /**< Y component of a vector. */
                float z; /**< Z component of a vector. */
            } xyz;       /**< X, Y, Z representation of a vector. */
            float v[3];  /**< Array representation of a vector. */
        } float3_t;

        struct imu_data
        {
            // acc timestamp and gyro timestamp is nearly the same
            //      thus we use acc timestamp.

            uint64_t timestamp_usec; /**< Timestamp of the accelerometer in microseconds. */
            float3_t gyro_sample; /**< Gyro sample in radians per second. */
            float3_t acc_sample;  /**< Accelerometer sample in meters per second squared. */
        };
    
    public:
        AzureKinectCamera() noexcept;

        ~AzureKinectCamera() noexcept;

        void startCamera();

        // Get (1) rgb (2) xyz (3) imu data
        // This function should be call around 30 fps to get all data
        void getCurrData( cv::Mat& rgb_output, \
                          cv::Mat& xyz_output, \
                          cv::Mat& ir_output, \
                          uint16_t& timestamp,\
                          std::vector<AzureKinectCamera::imu_data>& imu_outputs )
                    
        const std::string getModelName() const;

        // Get RGB image intrinsic matrix
        // ( cx, cy, fx, fy )
        cv::Vec4d getColorIntrinsics() const;

        // Get RGB image size
        // (width, height)
        cv::Size getImageSize() const;

        int getHeight() const;

        int getWidth() const;

    private:
        void freeResource();

    private:
        k4a_device_t device;
        k4a_capture_t capture;
        k4a_calibration_t calibration;
        k4a_transformation_t transformation;
        k4a_device_configuration_t camera_config;
        // cx, cy, fx, fy
        cv::Vec4d intrinsic;
        int img_width, img_height;
    };

}