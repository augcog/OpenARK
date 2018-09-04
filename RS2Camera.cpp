#include "stdafx.h"
#include "Version.h"
#include "RS2Camera.h"
#include "Visualizer.h"

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

		// get updated intrinsics
		rgbIntrinsics = new rs2_intrinsics();
		d2rExtrinsics = new rs2_extrinsics();
		r2dExtrinsics = new rs2_extrinsics();
		const std::vector<rs2::stream_profile> & stream_profiles = profile.get_streams();

		rs2::stream_profile rgbProfile;
		if (useRGBStream) rgbProfile = profile.get_stream(RS2_STREAM_COLOR);
		else rgbProfile = profile.get_stream(RS2_STREAM_INFRARED);

		rs2::stream_profile depthProfile = profile.get_stream(RS2_STREAM_DEPTH);
		*reinterpret_cast<rs2_intrinsics *>(rgbIntrinsics) =
			rgbProfile.as<rs2::video_stream_profile>().get_intrinsics();
		*reinterpret_cast<rs2_intrinsics *>(depthIntrinsics) =
			depthProfile.as<rs2::video_stream_profile>().get_intrinsics();
		*reinterpret_cast<rs2_extrinsics *>(d2rExtrinsics) = depthProfile.get_extrinsics_to(rgbProfile);
		*reinterpret_cast<rs2_extrinsics *>(r2dExtrinsics) = rgbProfile.get_extrinsics_to(depthProfile);
		scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
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
		rs2::frameset data;

		try {
			data = pipe->wait_for_frames();

			rs2::frame depth = data.first(RS2_STREAM_DEPTH);
			if (useRGBStream) {
				rs2::frame color = data.first(RS2_STREAM_COLOR);
				memcpy(rgb_map.data, color.get_data(), 3 * width * height);
				project(depth, color, xyz_map, rgb_map);
			}
			else {
				rs2::frame ir = data.first(RS2_STREAM_INFRARED);
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


	/*
	// project depth map to xyz coordinates relative to RGB/IR image
	void RS2Camera::project(const rs2::frame & depth_frame, const rs2::frame & rgb_frame, cv::Mat & xyz_map, cv::Mat & rgb_map) {
	const uint16_t * depth_data = (const uint16_t *)depth_frame.get_data();

	if (!depthIntrinsics || !rgbIntrinsics || !d2rExtrinsics) return;
	rs2_intrinsics * dIntrin = reinterpret_cast<rs2_intrinsics *>(depthIntrinsics);
	rs2_intrinsics * rIntrin = reinterpret_cast<rs2_intrinsics *>(rgbIntrinsics);
	rs2_extrinsics * drExtrin = reinterpret_cast<rs2_extrinsics *>(d2rExtrinsics);

	const uint16_t * srcPtr;
	cv::Vec3f * destPtr;
	float srcPixel[2], srcPixelTL[2], srcPixelBR[2], tmp1[3], tmp2[3];
	float destXYZ[3], tlIJ[2], brIJ[2];

	memset(xyz_map.data, 0, 12 * width * height);

	for (int r = 0; r < height; ++r)
	{
	srcPtr = depth_data + r * dIntrin->width;
	srcPixel[1] = r;
	srcPixelTL[1] = (float)r - 0.5f;
	srcPixelBR[1] = (float)r + 0.5f;

	for (int c = 0; c < width; ++c)
	{
	if (srcPtr[c] == 0) continue;

	// find central coordinates in destination 3D space
	srcPixel[0] = c;
	rs2_deproject_pixel_to_point(destXYZ, dIntrin, srcPixel, srcPtr[c] * scale);

	// find bounding box in destination 2D space
	srcPixelTL[0] = (float)c - 0.5f;
	srcPixelBR[0] = (float)c + 0.5f;
	rs2_deproject_pixel_to_point(tmp1, dIntrin, srcPixelTL, srcPtr[c] * scale);
	rs2_transform_point_to_point(tmp2, drExtrin, tmp1);
	rs2_project_point_to_pixel(tlIJ, rIntrin, tmp2);
	rs2_deproject_pixel_to_point(tmp1, dIntrin, srcPixelBR, srcPtr[c] * scale);
	rs2_transform_point_to_point(tmp2, drExtrin, tmp1);
	rs2_project_point_to_pixel(brIJ, rIntrin, tmp2);

	int tlX = std::floor(tlIJ[0]), tlY = std::floor(tlIJ[1]);
	int brX = std::ceil(brIJ[0]), brY = std::ceil(brIJ[1]);
	for (int y = tlY; y < brY; ++y) {
	if (y < 0 || y >= height) continue;
	destPtr = xyz_map.ptr<cv::Vec3f>(y);
	if (!destPtr) continue;
	for (int x = tlX; x < brX; ++x) {
	if (x < 0 || x >= width) continue;
	cv::Vec3f & vec = destPtr[x];
	if (vec[2] < 0.001) {
	memcpy(&vec, destXYZ, 3 * sizeof(float));
	}
	else {
	for (int i = 0; i < 3; ++i) {
	vec[i] *= 0.5;
	vec[i] += destXYZ[i] * 0.5;
	}
	}
	}
	}
	}
	}
	}
	*/
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

		ASSERT(list.size() > 0, "No camera detected.");
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

		ASSERT(this->depthIntrinsics, "FATAL: Camera has no depth stream!");
		width = depthIntrinsics->width;
		height = depthIntrinsics->height;

		config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16);
		if (useRGBStream) config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8);
		else config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8);
	}
}