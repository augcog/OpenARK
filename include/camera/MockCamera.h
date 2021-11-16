#pragma once

#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include "Version.h"
#include "camera/DepthCamera.h"

namespace ark {
    /**
     * This class defines the behavior of a camera that reads from a data file rather than a live camera
    **/
    class MockCamera : public DepthCamera
    {
    public:
        explicit MockCamera(const char* path);

        int getHeight() const override;

        int getWidth() const override;

        bool hasRGBMap() const override;

        /** Get the current timestamp in us; if unavailable, returns -1 */
        long long getTimestamp() const;

        /** Get the last frame time in us; if unavailable, returns -1 */
        long long getDeltaT() const;

        std::vector<cv::Point>& getJoints();
        std::vector<cv::Point> getJoints() const;

        ~MockCamera();

        bool hasNext() const;

    protected:
        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
            cv::Mat & amp_map, cv::Mat & flag_map) override;

    private:
        int height;
        int width;
        std::deque<std::string> depth_files;
        std::deque<std::string> rgb_files;
        std::deque<std::string> joint_files;
        std::deque<long long> timestamps;
        std::vector<cv::Point> joints;
        long long timestamp, deltaT;

        // camera intrinsics, if available
        double intr_fx, intr_fy, intr_cx, intr_cy = -1.;
    };
}
