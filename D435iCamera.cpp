#include "stdafx.h"
#include "Version.h"
#include "D435iCamera.h"
#include "Visualizer.h"

#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark {

    D435iCamera::D435iCamera(): D435iCamera(CameraParameter()){}

    D435iCamera::D435iCamera(const CameraParameter &parameter): 
        cameraParameter(parameter), last_ts_g(0), kill(false) {
        //Setup camera
        //TODO: Make read from config file


		//rs400::advanced_mode advanced_device(camera.getDevice());
		//auto depth_table = advanced_device.get_depth_table();
		//depth_table.depthClampMax = 1300; // 1m30 if depth unit at 0.001
		//advanced_device.set_depth_table(depth_table);


        rs2::context ctx;
        device = ctx.query_devices().front();
        width = cameraParameter.width;
        height = cameraParameter.height;
        //Setup configuration
        const auto irDepthFps = cameraParameter.irDepthFps;
        config.enable_stream(RS2_STREAM_DEPTH,-1,width, height,RS2_FORMAT_Z16,irDepthFps);
        config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, irDepthFps);
        config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, irDepthFps);
        config.enable_stream(RS2_STREAM_COLOR, -1, width, height, RS2_FORMAT_RGB8, irDepthFps);
        const auto imuFps = cameraParameter.imuFps;
        // TODO: check if fps of acc is neccesary to be different from gyro
        motion_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,imuFps+50);
        motion_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,imuFps);
        //Need to get the depth sensor specifically as it is the one that controls the sync funciton
        depth_sensor = new rs2::depth_sensor(device.first<rs2::depth_sensor>());

        scale = depth_sensor->get_option(RS2_OPTION_DEPTH_UNITS);




		//depth_sensor->set_option(RS2_OPTION_MAX_DISTANCE, 16.0f);
			
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
        depth_sensor->set_option(RS2_OPTION_EMITTER_ENABLED, cameraParameter.emitterPower);
        //start streaming
        pipe = std::make_shared<rs2::pipeline>();
        rs2::pipeline_profile selection = pipe->start(config);
        //get the depth intrinsics (needed for projection to 3d)
        auto depthStream = selection.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>();
        depthIntrinsics = depthStream.get_intrinsics();
		colorIntrinsics = selection.get_stream(RS2_STREAM_COLOR)
			.as<rs2::video_stream_profile>().get_intrinsics();

        motion_pipe = std::make_shared<rs2::pipeline>();
        rs2::pipeline_profile selection_motion = motion_pipe->start(motion_config);

        if (RS2_API_MAJOR_VERSION > 2 || RS2_API_MAJOR_VERSION == 2 && RS2_API_MINOR_VERSION >= 22) {
            
            auto dev = selection.get_device();
            auto sensors = dev.query_sensors();

            auto dev_motion = selection_motion.get_device();
            auto sensors_motion = dev_motion.query_sensors();

            int global_time_option = -1;
            string match = "Global Time Enabled";

            for (int i = 0; i < rs2_option::RS2_OPTION_COUNT; i++) {
                if (!strcmp(rs2_option_to_string((rs2_option)i), match.c_str())) {
                    global_time_option = i;
                    break;
                }
            }

            if (global_time_option == -1) {
                cout << "Couldn't find Global Time Enabled Option" << endl;
            }

            for (auto sensor: sensors) {
                sensor.set_option((rs2_option)global_time_option, false);
            }   

            for (auto sensor: sensors_motion) {
                sensor.set_option((rs2_option)global_time_option, false);
            }
        } 
        align_to_color = new rs2::align(RS2_STREAM_COLOR);
        imuReaderThread_ = std::thread(&D435iCamera::imuReader, this);
    }

	std::vector<float> D435iCamera::getColorIntrinsics() {
		return std::vector<float>{colorIntrinsics.fx, colorIntrinsics.fy, colorIntrinsics.ppx, colorIntrinsics.ppy};
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
        float imu_rate = static_cast<float>(cameraParameter.imuFps);
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

            std::cout << "getting camera frames" << std::endl;
            fflush(stdout);

            // Get frames from camera
            auto frames = pipe->wait_for_frames();
            auto infrared = frames.get_infrared_frame(1);        
            auto infrared2 = frames.get_infrared_frame(2);
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            std::cout << "kinda got frames" << std::endl;
            fflush(stdout);

            // Store ID for later
            frame.frameId_ = depth.get_frame_number();
            if(depth.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)){
                frame.timestamp_= depth.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)*1e3;
                //std::cout << "Image: " << std::fixed << frame.timestamp_/1e3 << std::endl;
            
            }else{
                std::cout << "No Metadata" << std::endl;
                fflush(stdout);
            }

            // Convert infrared frame to opencv
            if (frame.images_[0].empty()) frame.images_[0] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images_[0].data, infrared.get_data(),width * height);

            if (frame.images_[1].empty()) frame.images_[1] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images_[1].data, infrared2.get_data(),width * height);


            if (frame.images_[2].empty()) frame.images_[2] = cv::Mat(cv::Size(width,height), CV_32FC3);
            project(depth, frame.images_[2]);
            frame.images_[2] = frame.images_[2]*scale; //depth is in mm by default

			auto aligned_frames = align_to_color->process(frames);
			auto aligned_depth = aligned_frames.get_depth_frame();
			
			if (frame.images_[4].empty()) frame.images_[4] = cv::Mat(cv::Size(width, height), CV_16UC1, (void*)aligned_depth.get_data(), cv::Mat::AUTO_STEP);

            if (frame.images_[3].empty()) frame.images_[3] = cv::Mat(cv::Size(width,height), CV_8UC3);
            std::memcpy( frame.images_[3].data, color.get_data(),3 * width * height);

			//FILTER OUT ALL POINTS CLOSER THAN min_dist AND FARTHER THAN max_dist
			int min_dist = 200;
			int max_dist = 6000;
			for (int i = 0; i < width; i++) {
				for (int k = 0; k < height; k++) {
					if (frame.images_[4].at<uint16_t>(k, i) < min_dist || frame.images_[4].at<uint16_t>(k, i) > max_dist) { 
						frame.images_[4].at<uint16_t>(k, i) = 0;
					}
				}
			}

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
