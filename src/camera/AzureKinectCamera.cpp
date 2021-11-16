/* 
 * Azure Kinect Camera Class Implementation for Hand & Avatr
 * 
 * Alex Yu ( alexyu99126@gmail.com ) 2019
 * Xiao Song ( xiaosx@berkeley.edu ) 2021
 * 
 * Change Log
 * 2021-10-25: optimize the implementation.
 */

#include <iostream>
#include <cmath> // nanf
#include <k4a/k4a.h> // openark api
#include <opencv2/core.hpp>
#include "openark/stdafx.h"
#include "openark/camera/AzureKinectCamera.h"

/** Azure Kinect Cross-Platform Depth Camera Backend **/
namespace ark 
{
    AzureKinectCamera::AzureKinectCamera( ) noexcept : \
                                        device(NULL), \
                                        capture(NULL), \
                                        transformation(NULL), \
                                        xy_lookup_table(NULL) \
    {
        // Count number of k4a device to ensure there are connected device
        if ( k4a_device_get_installed_count() == 0 )
        {
            std::cerr << "Fatal: No Azure Kinect (K4A) devices found" << std::endl ;
            badInputFlag = true;
            return;
        }

        // Ensure camera can be open properly
        if ( K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
        {
            std::cerr << "Fatal: Failed to open Azure Kinect (K4A) device" << std::endl ;
            badInputFlag = true;
            return;
        }

        /* Camera configuration setting */
        camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        // currently only fixed frame rate is supported
        camera_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        // The Azure Kinect device does not natively capture in this format. 
        // Requesting images of this format requires additional computation in the API.
        camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; 
        // 1280 * 720  16:9
        camera_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        // 1024 * 1024 wide field of view depth without binned
        camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        // ensures that depth and color images are both available in the capture
        // objects may be produced only a single image when the corresponding image is dropped.
        camera_config.synchronized_images_only = true; 

        /* Get calibration information */
        if (K4A_RESULT_SUCCEEDED != \
            k4a_device_get_calibration(device, \
                                       camera_config.depth_mode, \
                                       camera_config.color_resolution, \
                                       &calibration))
        {
            std::cerr << "Fatal: Failed to get Azure Kinect (K4A) device calibration" << std::endl;;
            badInputFlag = true;
            return;
        }

        /* Set the intrinsics */
        auto& color_camera_intrin = calibration.color_camera_calibration.intrinsics.parameters.param;
        intrinsic[0] = color_camera_intrin.fx;
        intrinsic[1] = color_camera_intrin.cx;
        intrinsic[2] = color_camera_intrin.fy;
        intrinsic[3] = color_camera_intrin.cy;

        image_width = calibration.color_camera_calibration.resolution_width;
        image_height = calibration.color_camera_calibration.resolution_height;

        // Create depth/coor transformation
        transformation = k4a_transformation_create(&calibration);

        /* Create lookup table for depth to point cloud transformation */
        // If using the API provided by the k4a skd, 4 memory I/O is required
        //      read from depth, read from lookup table, 
        //      write to point cloud int16, write to point cloud float32
        // To reduce overall memory I/O cost (which is the bottleneck of this class)
        //      we use a customized lookup table and only incur 3 memory I/O
        //      read from depth, read from loopup table, write to point float float32
        // Building & use lookup table mostly follow 
        //      /Azure-Kinect-Sensor-SDK/examples/fastpointcloud/main.cpp
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        image_width,
                        image_height,
                        image_width * (int)sizeof(k4a_float2_t),
                        &xy_lookup_table);

        k4a_float2_t *lookup_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_lookup_table);
        k4a_float2_t p;
        k4a_float3_t ray;
        int valid;

        for (int y = 0, idx = 0; y < image_height; y++)
        {
            p.xy.y = (float)y;
            for (int x = 0; x < image_width; x++, idx++)
            {
                p.xy.x = (float)x;

                k4a_calibration_2d_to_3d(
                    &calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, \
                    K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);

                if (valid)
                {
                    lookup_table_data[idx].xy.x = ray.xyz.x / 1000.0f;
                    lookup_table_data[idx].xy.y = ray.xyz.y / 1000.0f;
                }
                else
                {
                    lookup_table_data[idx].xy.x = std::numeric_limits<float>::quiet_NaN(); //nanf("");
                    lookup_table_data[idx].xy.y = std::numeric_limits<float>::quiet_NaN(); //nanf("");
                }
            }
        }

        // Start camera
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &camera_config))
        {
            std::cerr << "Fatal: Failed to start Azure Kinect camera" << std::endl;;
            badInputFlag = true;
            return;
        }
    }

    AzureKinectCamera::~AzureKinectCamera() 
    {
        if (device != NULL)
        {
            k4a_device_stop_cameras(device);
            k4a_device_stop_imu(device);
            k4a_device_close(device);
        }
        
        if (transformation != NULL)
        {
            k4a_transformation_destroy(transformation);
        }
    }

    const std::string AzureKinectCamera::getModelName() const 
    {
        return std::string{"Azure Kinect"};
    }

    int AzureKinectCamera::getWidth() const 
    {
		return image_width;
    }

    int AzureKinectCamera::getHeight() const 
    {
		return image_height;
    }

	const DetectionParams::Ptr & AzureKinectCamera::getDefaultParams() const 
    {
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

	bool AzureKinectCamera::hasRGBMap() const 
    {
        return true;
	}

    uint64_t AzureKinectCamera::getTimestamp() const
    {
        return timestamp;
    }

    cv::Vec4d AzureKinectCamera::getCalibIntrinsics() const
    {
        return intrinsic;
    }

    // Get the latest data frame. 
    // This function is called by 
    //  thread DepthCamera::captureThreadingHelper 
    //      -> DepthCamera::nextFrame 
    //      -> swapBuffers() change pervious buffer with current buffer
    //
    // Arguments:
    //   size of ( width, height ) with 0 underlying elements
    //   xyz_output : CV_32FC3 
    //   rgb_output : CV_8UC3
    //   ir_map     : CV_8U
    //   amp_map    : CV_32F
    //   flag_map   : CV_8U
    void AzureKinectCamera::update(cv::Mat & xyz_output, \
                                   cv::Mat & rgb_output, \
                                   cv::Mat & ir_map, \
                                   cv::Mat & amp_map, \
                                   cv::Mat & flag_map )
    {
        // Runtime resource
        k4a_image_t image_sample = NULL;
        k4a_image_t depth_sample = NULL;
        k4a_image_t depth_sample_transformed_img_coord = NULL;
        k4a_image_t depth_sample_transformed_point_cloud = NULL;
    
        // Capture depth & image data
        capture = NULL;
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cerr << "Warning: Timed out waiting for a capture from Azure Kinect" << std::endl;;
                badInputFlag = true;
                k4a_capture_release(capture);
                return;
            case K4A_WAIT_RESULT_FAILED:
                std::cerr << "Warning: Failed to read a capture from Azure Kinect" << std::endl;;
                badInputFlag = true;
                k4a_capture_release(capture);
                return;
        }

        // Extract depth from capture
        depth_sample = k4a_capture_get_depth_image(capture);
        if ( !depth_sample ) 
        {
            std::cerr << "Warning: Failed to get depth image from Azure Kinect capture" << std::endl;;
            xyz_output = xyzMap.clone();
            rgb_output = rgbMap.clone();
            k4a_capture_release(capture);
            return;
        }

        // Extract BGRA/GRAY from capture
        image_sample = k4a_capture_get_color_image(capture);
        if (image_sample == NULL) 
        {
            std::cerr << "Warning: Failed to get color image from Azure Kinect capture" << std::endl;;
            xyz_output = xyzMap.clone();
            rgb_output = rgbMap.clone();
            k4a_image_release( depth_sample );
            k4a_capture_release( capture );
            return;
        }
        badInputFlag = false;

        // Extract device (the camera) time stamp & convert to nanoseconds
        timestamp = k4a_image_get_timestamp_usec(depth_sample) * 1e3;

        // Align depth to color image
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                    image_width,
                                                    image_height,
                                                    image_width * (int)sizeof(uint16_t), 
                                                    &depth_sample_transformed_img_coord))
        {
            std::cerr << "Failed to create transformed depth image" << std::endl;
            k4a_image_release( depth_sample );
            k4a_image_release( image_sample );
            k4a_image_release( depth_sample_transformed_img_coord );
            k4a_capture_release( capture );
            return;
        }

        if (K4A_RESULT_SUCCEEDED != \
            k4a_transformation_depth_image_to_color_camera(transformation, depth_sample, depth_sample_transformed_img_coord) )
        {
            std::cerr << "Warning: Failed to compute transformed depth image" << std::endl;
            k4a_image_release( depth_sample );
            k4a_image_release( image_sample );
            k4a_image_release( depth_sample_transformed_img_coord );
            k4a_capture_release( capture );
            return;
        }

        // Project depth to point cloud using the pre-build lookup table
        uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_sample_transformed_img_coord);
        k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_lookup_table);
        k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)xyz_output.data;

        for (int i = 0; i < image_width * image_height; i++)
        {
            if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
            {
                float depth_i = (float)depth_data[i];
                point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * depth_i;
                point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * depth_i;
                point_cloud_data[i].xyz.z = depth_i / 1000.0f;
            }
            else
            {
                point_cloud_data[i].xyz.x = 0; //nanf("");
                point_cloud_data[i].xyz.y = 0; //nanf("");
                point_cloud_data[i].xyz.z = 0; //nanf("");
            }
        }

        // Copy RGB result to output
        // Create a cv:::Mat using the k4a_image_t image buffer, there is no memory copy in this step
        // underlying data of image_sample is of format RGB Packed with no padding 
        // https://stackoverflow.com/questions/57222190/how-to-convert-k4a-image-t-to-opencv-matrix-azure-kinect-sensor-sdk
        cv::Mat rgba_output(k4a_image_get_height_pixels(image_sample), \
                            k4a_image_get_width_pixels(image_sample), \
                            CV_8UC4, \
                            k4a_image_get_buffer(image_sample) );

        // Convert RGBA to BGR using opencv
        // openCV use vectorize implementation with -O2 -O3. So it's faster then element wise operation
        // User may reuse the rgb_output variable to avoid allocate new memory every time
        cv::cvtColor(rgba_output, rgb_output, cv::COLOR_RGBA2RGB);

        // Release buffer
        k4a_image_release( depth_sample );
        k4a_image_release( image_sample );
        k4a_image_release( depth_sample_transformed_img_coord );
        k4a_capture_release( capture );
    }
}
