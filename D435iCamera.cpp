#include "stdafx.h"
#include "Version.h"
#include "D435iCamera.h"
#include "Visualizer.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark {
    D435iCamera::D435iCamera():
        last_ts_g(0), kill(false) {
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
        config.enable_stream(RS2_STREAM_COLOR, -1, width, height, RS2_FORMAT_RGB8, 30);
        motion_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,250);
        motion_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,200);
        imu_rate=200; //setting imu_rate to be the gyro rate since we are using the gyro timestamps
        //Need to get the depth sensor specifically as it is the one that controls the sync funciton
        depth_sensor = new rs2::depth_sensor(device.first<rs2::depth_sensor>());

        scale = depth_sensor->get_option(RS2_OPTION_DEPTH_UNITS);
        rs2::sensor color_sensor = device.query_sensors()[1];

        color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,false);

    }

    D435iCamera::~D435iCamera() {
        try {
            kill=true;
            imuReaderThread_.join();
            pipe->stop();
            if(depth_sensor){
                delete depth_sensor;
                depth_sensor=nullptr;
            }
        }
        catch (...) {}
    }

    void D435iCamera::start(){
        //enable sync
        //depth_sensor->set_option(RS2_OPTION_INTER_CAM_SYNC_MODE,1);
        //depth_sensor->set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
        //start streaming
        pipe = std::make_shared<rs2::pipeline>();
        rs2::pipeline_profile selection = pipe->start(config);
        //get the depth intrinsics (needed for projection to 3d)
        auto depthStream = selection.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>();
        depthIntrinsics = depthStream.get_intrinsics();

        motion_pipe = std::make_shared<rs2::pipeline>();
        motion_pipe->start(motion_config);
        imuReaderThread_ = std::thread(&D435iCamera::imuReader, this);
    }

    void D435iCamera::imuReader(){
        while(!kill){
            auto frames = motion_pipe->wait_for_frames();
            auto fa = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F)
                .as<rs2::motion_frame>();
            auto fg = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F)
                .as<rs2::motion_frame>();

            double ts_g = fg.get_timestamp();
            if(ts_g != last_ts_g){
                last_ts_g=ts_g;
                // std::cout << "GYRO: " << ts_g / 1e2 << std::endl;
                // Get gyro measures
                rs2_vector gyro_data = fg.get_motion_data();

                // Get the timestamp of the current frame
                double ts_a = fa.get_timestamp();
                //std::cout << "ACCEL: " << ts_a  <<  std::endl;
                // Get accelerometer measures
                rs2_vector accel_data = fa.get_motion_data();

                ImuPair imu_out { double(ts_g)*1e6, //convert to nanoseconds, for some reason gyro timestamp is in centiseconds
                    Eigen::Vector3d(gyro_data.x,gyro_data.y,gyro_data.z),
                    Eigen::Vector3d(accel_data.x,accel_data.y,accel_data.z)};
                imu_queue_.enqueue(imu_out);
            }

        }

    }

    bool D435iCamera::getImuToTime(double timestamp, std::vector<ImuPair>& data_out){
        ImuPair imu_data;
        imu_data.timestamp=0;
        while((imu_data.timestamp+1e9/imu_rate)<timestamp){
            if(imu_queue_.try_dequeue(&imu_data)){
                data_out.push_back(imu_data);
            }
        }
        return true;

    };



    const std::string D435iCamera::getModelName() const {
        return "RealSense";
    }

    cv::Size D435iCamera::getImageSize() const
    {
        return cv::Size(width,height);
    }

    void D435iCamera::update(MultiCameraFrame & frame) {

        try {
            // Ensure the frame has space for all images
            frame.images_.resize(5);

            // Get frames from camera
            auto frames = pipe->wait_for_frames();
            auto infrared = frames.get_infrared_frame(1);        
            auto infrared2 = frames.get_infrared_frame(2);
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            // Store ID for later
            frame.frameId_ = depth.get_frame_number();
            if(depth.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)){
                frame.timestamp_= depth.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)*1e3;
                //std::cout << "Image: " << std::fixed << frame.timestamp_/1e3 << std::endl;
            
            }else{
                std::cout << "No Metadata" << std::endl;
            }

            // Convert infrared frame to opencv
            if (frame.images_[0].empty()) frame.images_[0] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images_[0].data, infrared.get_data(),width * height);

            if (frame.images_[1].empty()) frame.images_[1] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images_[1].data, infrared2.get_data(),width * height);

            if (frame.images_[2].empty()) frame.images_[2] = cv::Mat(cv::Size(width,height), CV_32FC3);
            project(depth, frame.images_[2]);
            frame.images_[2] = frame.images_[2]*scale; //depth is in mm by default

            if (frame.images_[3].empty()) frame.images_[3] = cv::Mat(cv::Size(width,height), CV_8UC3);
            std::memcpy( frame.images_[3].data, color.get_data(),3 * width * height);

            if (frame.images_[4].empty()) frame.images_[4] = cv::Mat(cv::Size(width,height), CV_16UC1);
            // 16 bits = 2 bytes
            std::memcpy(frame.images_[4].data, depth.get_data(),width * height * 2);

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
    void D435iCamera::project(const rs2::frame & depth_frame, cv::Mat & xyz_map) {
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

    const rs2_intrinsics &D435iCamera::getDepthIntrinsics() {
        return depthIntrinsics;
    }

    double D435iCamera::getDepthScale() {
        return scale;
    }
}
