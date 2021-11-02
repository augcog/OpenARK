#include "stdafx.h"
#include "Version.h"
#include <camera/RS2Camera.h>
#include "util/Visualizer.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark {
	RS2Camera::RS2Camera(bool use_rgb_stream) : align(RS2_STREAM_COLOR), useRGBStream(use_rgb_stream) {
		pipe = std::make_shared<rs2::pipeline>();

		query_intrinsics();
		badInputFlag = false;
		rs2::pipeline_profile profile = pipe->start(config);

		// Editing laser intensity
		rs2::device selected_device = profile.get_device();
		auto depth_sensor = selected_device.first<rs2::depth_sensor>();

		printf("\nNote: \nSet laser power and exposure in RS2Camera.cpp \n");

		if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
			// Use range * (percent float) to adjust range percentage wise
			// auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);

			float laser_power = 225.f;
			printf("Setting laser power to -> %f \n", laser_power);
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power);
		}

		// Editing exposure
		if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
			float exposure = 6000.f;
			printf("Setting exposure to -> %f \n", exposure);
			depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
		}

		// get updated intrinsics
		rgbIntrinsics = new rs2_intrinsics();
		d2rExtrinsics = new rs2_extrinsics();
		r2dExtrinsics = new rs2_extrinsics();
		const std::vector<rs2::stream_profile> & stream_profiles = profile.get_streams();

		rs2::stream_profile depthProfile = profile.get_stream(RS2_STREAM_DEPTH);
		
		scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
		rs2::stream_profile rgbProfile;
		if (useRGBStream) {
			rgbProfile = profile.get_stream(RS2_STREAM_COLOR);
			rs2::align align(RS2_STREAM_COLOR);
		}
		else {
			rgbProfile = profile.get_stream(RS2_STREAM_INFRARED);
		}
	}

	RS2Camera::~RS2Camera() {
		try {
			pipe->stop();
		}
		catch (...) {}

		if (depthIntrinsics) delete reinterpret_cast<rs2_intrinsics *>(depthIntrinsics);
		if (rgbIntrinsics) delete reinterpret_cast<rs2_intrinsics *>(rgbIntrinsics);
		if (d2rExtrinsics) delete reinterpret_cast<rs2_extrinsics *>(d2rExtrinsics);
		depthIntrinsics = rgbIntrinsics = d2rExtrinsics = nullptr;
	}

	const std::string RS2Camera::getModelName() const {
		return "RealSense";
	}

	int RS2Camera::getWidth() const {
		return width;
	}

	int RS2Camera::getHeight() const {
		return height;
	}

	const DetectionParams::Ptr & RS2Camera::getDefaultParams() const {
		if (!defaultParamsSet) {
			defaultParamsSet = true;
			defaultParams = std::make_shared<DetectionParams>();
			defaultParams->contourImageErodeAmount = 0;
			defaultParams->contourImageDilateAmount = 2;
			defaultParams->fingerCurveFarMin = 0.18;
			defaultParams->fingerLenMin = 0.025;
			defaultParams->handClusterInterval = 15;
			defaultParams->handClusterMaxDistance = 0.003;
			defaultParams->handSVMConfidenceThresh = 0.52;
			defaultParams->handClusterMinPoints = 0.015;
			defaultParams->planeFloodFillThreshold = 0.19;
			defaultParams->planeEquationMinInliers = 0.02;
			defaultParams->planeMinPoints = 0.02;
			defaultParams->planeCombineThreshold = 0.0019;
			defaultParams->normalResolution = 3;
			defaultParams->handRequireEdgeConnected = false;
		}
		return defaultParams;
	}

	bool RS2Camera::hasRGBMap() const {
		return useRGBStream;
	}

	bool RS2Camera::hasIRMap() const {
		return !useRGBStream;
	}

	void RS2Camera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
		cv::Mat & amp_map, cv::Mat & flag_map) {
		rs2::frameset frameset = pipe->wait_for_frames();
		try {
			if (useRGBStream) {
				auto processed = align.process(frameset);
				rs2::frame color = processed.first(RS2_STREAM_COLOR);
				rs2::frame depth = processed.get_depth_frame();
				memcpy(rgb_map.data, color.get_data(), 3 * width * height);
				project(depth, color, xyz_map, rgb_map);
			}
			else {
				rs2::frame depth = frameset.first(RS2_STREAM_DEPTH);
				rs2::frame ir = frameset.get_infrared_frame(1);
				memcpy(ir_map.data, ir.get_data(), width * height);
				project(depth, ir, xyz_map, ir_map);
			}

		}
		catch (std::runtime_error e) {
			// try reconnecting
			badInputFlag = true;
			pipe->stop();
			printf("Couldn't connect to camera, retrying in 0.5s...\n");
			boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
			query_intrinsics();
			pipe->start(config);
			badInputFlag = false;
			return;
		}		
	}

	// project depth map to xyz coordinates directly (faster and minimizes distortion, but will not be aligned to RGB/IR)
	void RS2Camera::project(const rs2::frame & depth_frame, const rs2::frame & rgb_frame, cv::Mat & xyz_map, cv::Mat & rgb_map) {
		const uint16_t * depth_data = (const uint16_t *)depth_frame.get_data();

		if (!depthIntrinsics || !rgbIntrinsics || !d2rExtrinsics) return;
		rs2_intrinsics * dIntrin = reinterpret_cast<rs2_intrinsics *>(depthIntrinsics);

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
				rs2_deproject_pixel_to_point(destXYZ, dIntrin, srcPixel, srcPtr[c] * scale);
				memcpy(&destPtr[c], destXYZ, 3 * sizeof(float));
			}
		}
	}

	void RS2Camera::query_intrinsics() {
		rs2_intrinsics * depthIntrinsics = new rs2_intrinsics();

		rs2::context ctx;
		rs2::device_list list = ctx.query_devices();

		ARK_ASSERT(list.size() > 0, "No camera detected.");
		const rs2::device & dev = list.front();
		const std::vector<rs2::sensor> sensors = dev.query_sensors();

		for (unsigned i = 0; i < sensors.size(); ++i) {
			const rs2::sensor & sensor = sensors[i];
			const std::vector<rs2::stream_profile> & stream_profiles = sensor.get_stream_profiles();

			for (unsigned j = 0; j < stream_profiles.size(); ++j) {
				const rs2::stream_profile & stream_profile = stream_profiles[j];
				const rs2_stream & stream_data_type = stream_profile.stream_type();
				const rs2_format & stream_format = stream_profile.format();

				if (stream_profile.is<rs2::video_stream_profile>()) {
					if (stream_data_type == RS2_STREAM_DEPTH && stream_format == RS2_FORMAT_Z16) {
						const rs2::video_stream_profile & prof = stream_profile.as<rs2::video_stream_profile>();
						*depthIntrinsics = prof.get_intrinsics();
						this->depthIntrinsics = depthIntrinsics;
						if (depthIntrinsics->height == PREFERRED_FRAME_H) break;
					}
				}
			}
			if (this->depthIntrinsics) break;
		}

		ARK_ASSERT(this->depthIntrinsics, "FATAL: Camera has no depth stream!");
		width = depthIntrinsics->width;
		height = depthIntrinsics->height;

		config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16);
		if (useRGBStream) config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8);
		else config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8);
	}
}