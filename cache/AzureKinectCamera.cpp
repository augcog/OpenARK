#include "stdafx.h"
#include "Version.h"
#include "openark/slam_camera/AzureKinectCamera.h"

#include <k4a/k4a.h>

/** Azure Kinect Cross-Platform Depth Camera Backend **/
namespace ark {
    AzureKinectCamera::AzureKinectCamera(uint32_t device_id, bool wide_fov_mode, bool use_1080p, double scale)
        : scale(scale) {
        if (!k4a_device_get_installed_count())
        {
            std::cerr << "Fatal: No Azure Kinect (K4A) devices found\n";
            badInputFlag = true;
            return;
        }

        k4a_device_t device;
        if (k4a_device_open(device_id, &device) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Fatal: Failed to open Azure Kinect (K4A) device\n";
            badInputFlag = true;
            return;
        }
        this->k4a_device = device;

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        if (wide_fov_mode) {
            config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else {
            config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        if (use_1080p) {
            config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        }
        else {
            config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        }
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;

        k4a_calibration_t calibration;
        if (k4a_device_get_calibration(device, config.depth_mode,
            config.color_resolution, &calibration) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Fatal: Failed to get Azure Kinect (K4A) device calibration\n";
            badInputFlag = true;
            return;
        }

        // Set the intrinsics
        auto& color_cam_intrin = calibration.color_camera_calibration.intrinsics.parameters.param;
        calib_intrin[0] = color_cam_intrin.fx;
        calib_intrin[1] = color_cam_intrin.cx;
        calib_intrin[2] = color_cam_intrin.fy;
        calib_intrin[3] = color_cam_intrin.cy;

        width = calibration.color_camera_calibration.resolution_width;
        height = calibration.color_camera_calibration.resolution_height;
        scaled_width = width * scale;
        scaled_height = height * scale;

        // Create depth/coor transformation
        auto transformation = k4a_transformation_create(&calibration);
        this->k4a_transformation = transformation;

        if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Fatal: Failed to start Azure Kinect camera\n";
            badInputFlag = true;
            return;
        }
        
        // Generate XY table cache
        k4a_image_t xy_table;
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            width, height, width * (int)sizeof(k4a_float2_t), &xy_table);

        auto table_data = reinterpret_cast<k4a_float2_t *>(k4a_image_get_buffer(xy_table));
        k4a_float2_t p;
        k4a_float3_t ray;
        int valid;
        for (int y = 0, idx = 0; y < height; ++y)
        {
            p.xy.y = static_cast<float>(y);
            for (int x = 0; x < width; ++x, ++idx)
            {
                p.xy.x = static_cast<float>(x);

                k4a_calibration_2d_to_3d(
                    &calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR,
                    K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);

                if (valid)
                {
                    table_data[idx].xy.x = ray.xyz.x / 1000.;
                    table_data[idx].xy.y = ray.xyz.y / 1000.;
                }
                else
                {
                    table_data[idx].xy.x = table_data[idx].xy.y = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        this->xy_table_cache = xy_table;
    }

    AzureKinectCamera::~AzureKinectCamera() {
        auto device = reinterpret_cast<k4a_device_t>(this->k4a_device);
        if (device != nullptr) {
            k4a_device_close(device);
        }
        auto transformation = reinterpret_cast<k4a_transformation_t>(this->k4a_transformation);
        if (transformation != nullptr) {
            k4a_transformation_destroy(transformation);
        }
        auto xy_table = reinterpret_cast<k4a_image_t>(this->xy_table_cache);
        if (xy_table != nullptr) {
            k4a_image_release(xy_table);
        }
    }

    const std::string AzureKinectCamera::getModelName() const {
        return "Azure Kinect";
    }

    int AzureKinectCamera::getWidth() const {
		return scaled_width;
    }

    int AzureKinectCamera::getHeight() const {
		return scaled_height;
    }

	const DetectionParams::Ptr & AzureKinectCamera::getDefaultParams() const {
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

	bool AzureKinectCamera::hasRGBMap() const {
        return true;
	}

    uint64_t AzureKinectCamera::getTimestamp()
    {
        return timestamp;
    }

    cv::Vec4d AzureKinectCamera::getCalibIntrinsics()
    {
        return calib_intrin;
    }

    void AzureKinectCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
        cv::Mat & amp_map, cv::Mat & flag_map) {
        auto device = reinterpret_cast<k4a_device_t>(this->k4a_device);
        auto xy_table = reinterpret_cast<k4a_image_t>(this->xy_table_cache);
        auto transformation = reinterpret_cast<k4a_transformation_t>(this->k4a_transformation);

        k4a_capture_t capture = NULL;
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            std::cerr << "Warning: Timed out waiting for a capture from Azure Kinect\n";
            badInputFlag = true;
            return;
        case K4A_WAIT_RESULT_FAILED:
            std::cerr << "Warning: Failed to read a capture from Azure Kinect\n";
            badInputFlag = true;
            return;
        }

        auto depth_image = k4a_capture_get_depth_image(capture);
        while (depth_image == NULL) {
            std::cerr << "Warning: Failed to get depth image from Azure Kinect capture\n";
            xyz_map = xyzMap.clone();
            rgb_map = rgbMap.clone();
            return;
        }

        auto color_image = k4a_capture_get_color_image(capture);
        while (color_image == NULL) {
            std::cerr << "Warning: Failed to get color image from Azure Kinect capture\n";
            xyz_map = xyzMap.clone();
            rgb_map = rgbMap.clone();
            k4a_image_release(depth_image);
            return;
        }
        badInputFlag = false;

        // Fill in color image (eliminate alpha channel)
        cv::Mat rgb_map_large(height, width, CV_8UC3);
        auto color_data = reinterpret_cast<cv::Vec4b *>(k4a_image_get_buffer(color_image));
        auto rgb_map_data = reinterpret_cast<Vec3b *>(rgb_map_large.data);

        for (int i = 0; i < width * height; ++i) {
            rgb_map_data[i][0] = color_data[i][0];
            rgb_map_data[i][1] = color_data[i][1];
            rgb_map_data[i][2] = color_data[i][2];
        }
        if (scale = 1.0) {
            rgb_map = rgb_map_large;
        }
        else {
            cv::resize(rgb_map_large, rgb_map, rgb_map.size(), 0., 0., CV_INTER_CUBIC);
        }

        // Align depth to color image
        k4a_image_t transformed_depth_image = NULL;
        int color_image_width = k4a_image_get_width_pixels(color_image);
        int color_image_height = k4a_image_get_height_pixels(color_image);
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width, color_image_height,
            color_image_width * (int)sizeof(uint16_t),
            &transformed_depth_image);

        if (k4a_transformation_depth_image_to_color_camera(transformation,
            depth_image, transformed_depth_image) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Warning: Failed to compute transformed depth image\n";
            k4a_image_release(depth_image);
            k4a_image_release(color_image);
            k4a_image_release(transformed_depth_image);
            return;
        }

        // Create XYZ map from transformed depth
        auto transformed_depth_data =
            reinterpret_cast<uint16_t *>(k4a_image_get_buffer(transformed_depth_image));
        auto xy_table_data = reinterpret_cast<k4a_float2_t *>(k4a_image_get_buffer(xy_table));
        cv::Mat xyz_map_large(height, width, CV_32FC3);
        auto xyz_map_data = reinterpret_cast<Vec3f *>(xyz_map_large.data);

        for (int i = 0; i < width * height; ++i)
        {
            if (transformed_depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
            {
                xyz_map_data[i][0] = xy_table_data[i].xy.x * (float)transformed_depth_data[i];
                xyz_map_data[i][1] = xy_table_data[i].xy.y * (float)transformed_depth_data[i];
                xyz_map_data[i][2] = (float)transformed_depth_data[i] / 1000.;
            }
            else
            {
                xyz_map_data[i] = 0;
            }
        }

        if (scale = 1.0) {
            xyz_map = xyz_map_large;
        }
        else {
            cv::resize(xyz_map_large, xyz_map, xyz_map.size(), 0., 0., CV_INTER_NN);
        }

        // get timestamp and convert to nanoseconds
        timestamp = k4a_image_get_timestamp_usec(depth_image) * 1e3;

        k4a_image_release(depth_image);
        k4a_image_release(color_image);
        k4a_image_release(transformed_depth_image);
        k4a_capture_release(capture);
    }
}
