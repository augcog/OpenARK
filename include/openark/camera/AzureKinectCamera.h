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
#include <Eigen/Core> // Eigen::aligned_allocator
#include <k4a/k4a.h> // k4a_device_t
#include "Types.h" // CameraParameter, MultiCameraFrame, ImuPair, CameraParameter

namespace ark
{

    // Design: currently we only support AzureKinect camera. 
    // Thus there's no need to derive from base class.
    class AzureKinectCamera
    {
    public:
        AzureKinectCamera();

        ~AzureKinectCamera();

        /*
         *
         *
         * 
         * 
         */
        void startCamera();
        
        /*
         * Get latest datas (frames) from sensor stream. 
         * 
         * 
         * 
         */
        void getCurrData(MultiCameraFrame::Ptr);

        std::string getModelName() const;

        /*
         * Intrinsic matrix of RGB Camera
         */
        std::vector<float> getColorIntrinsics();

        /*
         *
         *
         */
        cv::Size getImageSize() const override;

        double getDepthScale();

        /*
         * Get IMU data based on time stamp
         *
         * Eigen::aligned_allocator is used to align data that contain Eigen component
         *  this related to how C++98 - C++14 work with dynamic memory allocation.
         *  https://eigen.tuxfamily.org/dox/classEigen_1_1aligned__allocator.html
         *  https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html 
         *  https://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
         */

    private:
        k4a_device_t device;
        k4a_capture_t capture;
        k4a_calibration_t calibration;
        k4a_transformation_t transformation;
        // Time in milliseconds waiting for capture result to return. 
        // If set to 0, the k4a_device_get_capture will return without blocking
        // If set K4A_WAIT_INFINITE will block indefinitely until data avalibe
        // We set this value to 1000 by default
        const int32_t TIMEOUT_IN_MS;
        // cx, cy, fx, fy
        cv::Vec4d intrinsic;
        int image_resolution_width, image_resolution_height;
    };

}