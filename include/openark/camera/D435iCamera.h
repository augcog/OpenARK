#pragma once

#include <opencv2/core.hpp>
#include <thread>
#include <atomic>
#include <librealsense2/rs.hpp>
#include "openark/camera/CameraSetup.h"
#include "Version.h"
#include "openark/util/concurrency.h"

namespace ark {
    /**
    * Class defining the behavior of a generic Intel RealSense Camera using RSSDK2.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class D435iCamera : public CameraSetup
    {
    public:

        /**
        * Public constructor initializing the RealSense Camera.
        * @param use_rgb_stream if true, uses the RGB stream and disable the IR stream (which is on by default)
        *                       This results in a smaller field of view and has an appreciable performance cost.
        */
        explicit D435iCamera();

        D435iCamera(const CameraParameter &parameter);

        /**
        * Destructor for the RealSense Camera.
        */
        ~D435iCamera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /**
         * Get image size
         */
        cv::Size getImageSize() const override;

        /** 
         * Sets the external hardware sync ans starts the camera
         */
        void start() override;
        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(MultiCameraFrame::Ptr frame) override;

        bool getImuToTime(double timestamp, std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>>& data_out);

        std::vector<float> getColorIntrinsics() override;

        const rs2_intrinsics &getDepthIntrinsics();

        double getDepthScale();

    protected:

        /** Converts an D435 raw depth image to an ordered point cloud based on the current camera's intrinsics */
        void project(const rs2::frame & depth_frame, cv::Mat & xyz_map);

        /**
        * Reads data from the imu
        */
        void imuReader();

        std::shared_ptr<rs2::pipeline> pipe;
        std::shared_ptr<rs2::pipeline> motion_pipe;
		std::shared_ptr<rs2::pipeline> color_depth_pipe;
        rs2::config config;
        rs2::config motion_config;
		rs2::config color_depth_config;
        rs2::depth_sensor* depth_sensor;
        rs2::device device;
        rs2_intrinsics depthIntrinsics;
        std::thread imuReaderThread_;
        single_consumer_queue<ImuPair> imu_queue_;

        CameraParameter cameraParameter;
        int width, height;
        double scale;
        double last_ts_g;
        bool badInputFlag;
        std::atomic<bool> kill;

		rs2::align * align_to_color;
		rs2_intrinsics colorIntrinsics;

    };
}
