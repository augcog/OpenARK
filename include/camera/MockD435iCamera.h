#pragma once
// OpenCV Libraries
#include <ctime>
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>
#include <boost/filesystem.hpp>
#include <thread>
#include "concurrency.h"
#include <atomic>
#include <iostream>

// OpenARK Libraries
#include "CameraSetup.h"
#include "util/Util.h"
using boost::filesystem::path;
using std::ifstream;
namespace ark {
    /**
    * Mock camera for replaying data
    */
    class MockD435iCamera : public CameraSetup
    {
    public:

        /**
        * config the input dir
        */
        explicit MockD435iCamera(path dir);

        /**
        * Destructor
        */
        ~MockD435iCamera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /**
         * Get image size
         */
        cv::Size getImageSize() const override;

        /** 
         * Dummy method
         */
        void start() override;
        /**
        * get a frame per time
        */
        void update(MultiCameraFrame::Ptr frame) override;

        void project(const cv::Mat &depth_frame, cv::Mat &xyz_map);

        bool getImuToTime(double timestamp, std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>>& data_out);

        std::vector<ImuPair> getAllImu();

        std::vector<float> getColorIntrinsics() override;

    protected:

        cv::Mat loadImg(path filename);

        path dataDir;
        path imuTxtPath;
        path timestampTxtPath;
        path metaTxtPath;
        path intrinFilePath;
        path depthDir;
        path rgbDir;
        path infraredDir;
        path infrared2Dir;
        ifstream imuStream;
        ifstream timestampStream;
        rs2_intrinsics depthIntrinsics;
        int firstFrameId;
        int width, height;
        time_t startTime;
        double scale;
    };
}