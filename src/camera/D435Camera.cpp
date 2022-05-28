#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include "stdafx.h"
#include "Version.h"
#include "openark/camera/D435Camera.h"
#include "openark/util/Visualizer.h"

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark {
    D435Camera::D435Camera() {
        //Setup camera
        //TODO: Make read from config file
        rs2::context ctx;
        device = ctx.query_devices().front();
        width = 640;
        height = 480;
        //Setup configuration
        config.enable_stream(RS2_STREAM_DEPTH,-1,width, height,RS2_FORMAT_Z16,30);
        config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
        config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);
        //Reset device to ensure sync is off
        device.hardware_reset();
        //Give camera time to restart, this shouldn't be necessary but it is
        boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
        //Wait for the camera to finish restarting
        rs2::device_hub hub(ctx);
        device = hub.wait_for_device();
        //Need to get the depth sensor specifically as it is the one that controls the sync funciton
        depth_sensor = new rs2::depth_sensor(device.first<rs2::depth_sensor>());

        scale = depth_sensor->get_option(RS2_OPTION_DEPTH_UNITS);

    }

    D435Camera::~D435Camera() {
        try {
            pipe->stop();
            if(depth_sensor){
                delete depth_sensor;
                depth_sensor=nullptr;
            }
        }
        catch (...) {}
    }

    void D435Camera::start(){
        //enable sync
        depth_sensor->set_option(RS2_OPTION_INTER_CAM_SYNC_MODE,1);
        //depth_sensor->set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
        //start streaming
        pipe = std::make_shared<rs2::pipeline>();
        rs2::pipeline_profile selection = pipe->start(config);
        //get the depth intrinsics (needed for projection to 3d)
        auto depthStream = selection.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>();
        depthIntrinsics = depthStream.get_intrinsics();
    }

    const std::string D435Camera::getModelName() const {
        return "RealSense";
    }

    cv::Size D435Camera::getImageSize() const
    {
        return cv::Size(width,height);
    }

    void D435Camera::update(MultiCameraFrame & frame) {

        try {
            // Ensure the frame has space for all images
            frame.images_.resize(3);

            // Get frames from camera
            auto frames = pipe->wait_for_frames();
            auto infrared = frames.get_infrared_frame(1);        
            auto infrared2 = frames.get_infrared_frame(2);
            auto depth = frames.get_depth_frame();

            // Store ID for later
            frame.frameId_ = infrared.get_frame_number();

            // Convert infrared frame to opencv
            if (frame.images_[0].empty()) frame.images_[0] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images_[0].data, infrared.get_data(),width * height);

            if (frame.images_[1].empty()) frame.images_[1] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images_[1].data, infrared2.get_data(),width * height);

            if (frame.images_[2].empty()) frame.images_[2] = cv::Mat(cv::Size(width,height), CV_32FC3);
            project(depth, frame.images_[2]);
            frame.images_[2] = frame.images_[2]*scale; //depth is in mm by default


        } catch (std::runtime_error e) {
            // Try reconnecting
            badInputFlag = true;
            pipe->stop();
            printf("Couldn't connect to camera, retrying in 0.5s...\n");
            boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
            //query_intrinsics();
            pipe->start(config);
            badInputFlag = false;
            return;
        }
    }

    // project depth map to xyz coordinates directly (faster and minimizes distortion, but will not be aligned to RGB/IR)
    void D435Camera::project(const rs2::frame & depth_frame, cv::Mat & xyz_map) {
        const uint16_t * depth_data = (const uint16_t *)depth_frame.get_data();

        rs2_intrinsics * dIntrin = &depthIntrinsics;

        const uint16_t * srcPtr;
        cv::Vec3f * destPtr;
        float srcPixel[2], destXYZ[3];

        for (int r = 0; r < height; ++r)
        {
            srcPtr = depth_data + r * dIntrin->width;
            destPtr = xyz_map.ptr<Vec3f>(r);
            srcPixel[1] = r;

            for (int c = 0; c < width; ++c)
            {
                if (srcPtr[c] == 0) {
                    memset(&destPtr[c], 0, 3 * sizeof(float));
                    continue;
                }
                srcPixel[0] = c;
                rs2_deproject_pixel_to_point(destXYZ, dIntrin, srcPixel, srcPtr[c]);
                memcpy(&destPtr[c], destXYZ, 3 * sizeof(float));
            }
        }
    }

}
