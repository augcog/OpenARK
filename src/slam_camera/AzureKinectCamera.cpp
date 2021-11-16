/*
 * Azure Kinect Camera Class Implementation
 *
 * Xiao Song ( xiaosx@berkeley.edu )
 * 
 * Azure Kinect DK document https://docs.microsoft.com/en-us/azure/Kinect-dk/
 * Azure Kinect SDK Document https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/index.html
 * Azure Kinect SDK github repo https://github.com/microsoft/Azure-Kinect-Sensor-SDK
 * 
 * A brief outline of Azure Kinect Sensor SDK support
 * 1. Depth camera access and mode control (a passive IR mode, plus wide and narrow field-of-view depth modes)
 * 2. Motion sensor (gyroscope and accelerometer) access
 * 3. RGB camera access and control (for example, exposure and white balance)
 * 4. Synchronized Depth-RGB camera streaming with configurable delay between cameras
 * 5. External device synchronization control with configurable delay offset between devices
 * 6. Camera frame meta-data access for image resolution, timestamp, etc.
 * 7. Device calibration data access
 * 
 * Update 2021-11-16 : this file is deprecated. Need to repair if needed.
 */

#include <vector>
#include <opencv2/core.hpp> // cv::Mat
#include <opencv2/imgproc.hpp> // cv::cvtColor
#include <k4a/k4a.h> // AzureKinect API
#include <cstring> // memcpy
#include "slam_camera/AzureKinectCamera.h"

// TODO add depth on & depth off support. Sometimes we don't need to collect depth information
// TODO change print error to print to google test

namespace ark
{
    // Delegate constructor call
    AzureKinectCamera::AzureKinectCamera() noexcept : \
        device(NULL), capture(NULL), \
        calibration(NULL), transformation(NULL) \
    {
        // Camera configruation
        camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        // currently only fixed frame rate is supported
        camera_config.camera_fps       = K4A_FRAMES_PER_SECOND_30;
        // RGBA 4 uint8
        // The Azure Kinect device does not natively capture in this format. 
        // Requesting images of this format requires additional computation in the API.
        camera_config.color_format     = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        // trun off  image
        //camera_config.color_format     = K4A_COLOR_RESOLUTION_OFF;
        // currently only fixed ratio 1280 * 720  16:9 is supported
        camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        // NOTICE: depth size is not same as RGB size
        // 512 * 512 wide field of view depth with binned (combine 2*2 pixel into 1)
        //camera_config.depth_mode       = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        // 1024 * 1024 wide field of view depth without binned
        //camera_config.depth_mode       = K4A_DEPTH_MODE_WFOV_UNBINNED;
        // 640 * 576 narrow field of view depth without binned
        camera_config.depth_mode       = K4A_DEPTH_MODE_NFOV_UNBINNED;
        // trun off depth
        //camera_config.depth_mode       = K4A_DEPTH_MODE_OFF;
        // ensures that depth and color images are both available in the capture
        // objects may be produced only a single image when the corresponding image is dropped.
        camera_config.synchronized_images_only = true; 

    }

    AzureKinectCamera::~AzureKinectCamera() noexcept
    {
        freeResource();
    }

    void AzureKinectCamera::freeResource()
    {
        if (transformation != NULL)
        {
            k4a_transformation_destroy(transformation);
        }
        if (device != NULL)
        {
            k4a_device_stop_cameras(device);
            k4a_device_stop_imu(device);
            k4a_device_close(device);
        }
    }

    void AzureKinectCamera::startCamera()
    {
        // References:
        // Azure-Kinect-Sensor-SDK/examples/streaming/main.c
        // Azure-Kinect-Sensor-SDK/examples/transformation/main.cpp

        // Design: constructor should not throw error. Thus we do not put
        //      below check device count logic inside the constructor

        // Check number of k4a device
        if (k4a_device_get_installed_count() == 0)
        {
            printf("No K4A devices found\n");
            return this->freeResource();
        }

        // Open camera
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
        {
            printf("Failed to open device\n");
            return this->freeResource();
        }

        // Get calibration information
        if (K4A_RESULT_SUCCEEDED != \
            k4a_device_get_calibration(device, \
                                       camera_config.depth_mode, \
                                       camera_config.color_resolution, \
                                       &calibration))
        {
            printf("Failed to get calibration\n");
            return this->freeResource();
        }

        // Set Intrinsic matrix
        // Azure-Kinect-Sensor-SDK/examples/calibration/main.cpp
        auto& color_camera_intrin = calibration.color_camera_calibration.intrinsics.parameters.param;
        intrinsic.at(0) = color_camera_intrin.cx;
        intrinsic.at(1) = color_camera_intrin.cy;
        intrinsic.at(2) = color_camera_intrin.fx;
        intrinsic.at(3) = color_camera_intrin.fy;

        // Image resolution 
        img_width = calibration.color_camera_calibration.resolution_width;
        img_height = calibration.color_camera_calibration.resolution_height;

        // Set transformation
        // A xy look up table is pre-computed when calling this function. The xy look up table
        //      help speed up the computation time of depth to point cloud projection
        //
        // https://docs.microsoft.com/en-us/azure/kinect-dk/use-image-transformation#k4a_transformation_depth_image_to_point_cloud
        transformation = k4a_transformation_create(&calibration);

        // Start camera
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &camera_config))
        {
            printf("Failed to start device\n");
            return this->freeResource();
        }

        // Start IMU
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
        {
            printf("Failed to start imu\n");
            return this->freeResource();
        }
    }

    void AzureKinectCamera::getCurrData( cv::Mat& rgb_output, \
                                         cv::Mat& xyz_output, \
                                         cv::Mat& ir_output, \
                                         uint16_t& timestamp,\
                                         std::vector<AzureKinectCamera::imu_data>& imu_outputs )
    {
        // For faster speed, the rgb_output and xyz_output should already allocate correct height and width
        // User shuold call cv::Mat rgb_output(AzureKinectCamera.getSize(), CV_8UC4) to create rgb_output and pass in
        // User should call cv::Mat xyz_output(AzureKinectCamera.getSize(), CV_16C3) to create xyz_output and pass in
        if ( rgb_output.size[0] != img_width || 
             rgb_output.size[1] != img_height ||
             xyz_output.size[0] != img_width || 
             xyz_output.size[1] != img_height )
        {
            printf("Input memory is not allocated in correct size\n");
            return;
        }

        // Commonly we have around 6 imu sample for each camera frame.
        // Reserve enough memory space to avoid copy overhead
        imu_outputs.reserve(10);

        k4a_image_t image_sample = NULL;
        k4a_image_t depth_sample = NULL;
        k4a_image_t ir_sample = NULL;
        k4a_image_t depth_transformed_image_coordinate = NULL;
        k4a_image_t point_cloud_transformed_image_coordinate = NULL;
        k4a_imu_sample_t imu_sample = NULL;

        // Capture depth & image data
        //
        // Third parameter :
        //      Time in milliseconds waiting for capture result to return. 
        //      If set to 0, the k4a_device_get_capture will return without blocking
        //      If set K4A_WAIT_INFINITE will block indefinitely until data avalibe
        //      We set this value to 1000 by default
        //
        // Azure-Kinect-Sensor-SDK/examples/streaming/main.c
        capture = NULL; // reset for safety
        switch (k4a_device_get_capture(device, &capture, 1000))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a capture\n");
                goto ReleaseRunTimeResource;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                goto ReleaseRunTimeResource;
        }

        // Extract BGRA/GRAY from capture
        image_sample = k4a_capture_get_color_image(capture);
        if ( !image_sample )
        {
            printf("Failed to read image from capture\n");
            goto ReleaseRunTimeResource;
        }

        // Extract depth from capture
        depth_sample = k4a_capture_get_depth_image(capture);
        if ( !depth_sample )
        {
            printf("Failed to read depth from capture\n");
            goto ReleaseRunTimeResource;
        }

        // Extract infra-red from capture
        ir_sample = k4a_capture_get_ir_image( capture );
        if ( !ir_sample )
        {
            printf("Failed to read depth from capture\n");
            goto ReleaseRunTimeResource;
        }

        // Extract device (the camera) time stamp
        timestamp = k4a_image_get_timestamp_usec(image_sample)

        // Capture a imu sample
        // IMU data is generate faster (around 200 fps) than depth / color image (around 30 fps)
        // IMU sensor have internal queue. 
        // The API also has sufficient internal queuing to allow you to only check for samples 
        //      after each image capture is returned.
        // To retrieve all the currently queued IMU samples, 
        //      you can call k4a_device_get_imu_sample() with a timeout_in_ms of 0 in a loop until the 
        //      function returns K4A_WAIT_RESULT_TIMEOUT.
        //
        // Azure-Kinect-Samples/body-tracking-samples/floor_detector_sample/main.cpp
        // https://docs.microsoft.com/en-us/azure/kinect-dk/retrieve-imu-samples
        // https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___functions_ga8e5913b3bb94a453c7143bbd6e399a0e.html#ga8e5913b3bb94a453c7143bbd6e399a0e
        while( true )
        {
            switch (k4a_device_get_imu_sample(device, &imu_sample, 0))
            {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a imu sample\n");
                goto exit_loop;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a imu sample\n");
                goto ReleaseRunTimeResource;
            }

            // acc timestamp and gyro timestamp is nearly the same
            //      thus we use acc timestamp.
            imu_outputs.emplace_back({imu_sample.acc_timestamp_usec, \
                                      reinterpret_cast<float3_t>(imu_sample.gyro_sample) \
                                      reinterpret_cast<float3_t>(imu_sample.acc_sample) });
        }
        exit_loop:

        // Transform depth to color coordinate (uint16_t)
        // Allocate memory for transformed depth image.
        // depth is store using uint16_t
        //
        // Azure-Kinect-Sensor-SDK/examples/transformation/main.cpp
        //
        //  directely on top of the output variable
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                    img_width,
                                                    img_height,
                                                    img_width * (int)sizeof(uint16_t), 
                                                    &depth_transformed_image_coordinate))
        {
            printf("Failed to create transformed depth image\n");
            goto ReleaseRunTimeResource;
        }

        // Transform depth
        if (K4A_RESULT_SUCCEEDED != \
            k4a_transformation_depth_image_to_color_camera(transformation, \
                                                           depth_sample, \
                                                           depth_transformed_image_coordinate ))
        {
            printf("Failed to compute transformed depth image\n");
            goto ReleaseRunTimeResource;
        }

        // Reproject transformed depth from 2d to 3d (3 * int16_t)
        // IMPORTANT NOTE: currently the transformation output int16_t data representing the point cloud
        //    but the old API of D345iCamera uses float32_t. Three solutions to handle this problem. 
        //    (1) use opencv to convert HW3 int16_t matrix to HW3 float32_t matrix
        //    (2) manually compute output and store it in float32_t as Azure-Kinect-Sensor-SDK/examples/fastpointcloud/main.cpp did
        //    (3) use int16_t data instead
        /*
        // See below section on how to reuse existing memory and avoid memory create
        // Create point cloud buffer and then copy it to cv::Mat buffer
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                    img_width,
                                                    img_height,
                                                    img_width * 3 * (int)sizeof(int16_t),
                                                    &point_cloud_transformed_image_coordinate))
        {
            printf("Failed to create point cloud image\n");
            return false;
        }
        */

        // Create point cloud from opencv buffer. This reduce memory copy
        // Azure-Kinect-Sensor-SDK/tests/RecordTests/UnitTest/record_ut.cpp
        if ( K4A_RESULT_SUCCEEDED != \
             k4a_image_create_from_buffer( K4A_IMAGE_FORMAT_CUSTOM,
                                           img_width,
                                           img_height,
                                           img_width * 3 * (int)sizeof(int16_t),
                                           reinterpret_cast<uint8_t*> xyz_output.data,
                                           img_width * img_width * 3 * (int)sizeof(int16_t),
                                           nullptr,
                                           nullptr,
                                           point_cloud_transformed_image_coordinate ))
        {
            printf("Failed to create point cloud buffer from cv::Mat memory\n");
            goto ReleaseRunTimeResource;
        }

        // K4A_CALIBRATION_TYPE_COLOR is used because depth is reproject to camera coordinate first
        // Each pixel of the xyz_image (point_cloud_transformed_image_coordinate) consists of three int16_t values
        //
        // Azure-Kinect-Sensor-SDK/examples/transformation/main.cpp
        // https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___functions_ga7385eb4beb9d8892e8a88cf4feb3be70.html
        if (K4A_RESULT_SUCCEEDED != \
            k4a_transformation_depth_image_to_point_cloud(transformation,
                                                          depth_transformed_image_coordinate,
                                                          K4A_CALIBRATION_TYPE_COLOR,
                                                          point_cloud_transformed_image_coordinate))
        {
            printf("Failed to compute point cloud\n");
            goto ReleaseRunTimeResource;
        }

        // Copy result to output
        // Create a cv:::Mat using the k4a_image_t image buffer, there is no memory copy in this step
        // https://stackoverflow.com/questions/57222190/how-to-convert-k4a-image-t-to-opencv-matrix-azure-kinect-sensor-sdk
        cv::Mat rgba_output(img_height, \
                            img_width, \
                            CV_8UC4, \
                            reinterpret_cast<void*>k4a_image_get_buffer(image_sample), \
                            cv::Mat::AUTO_STEP);

        // Convert RGBA to BGR using opencv
        // openCV use vectorize implementation with -O2 -O3. So it's faster then element wise operation
        // User may reuse the rgb_output variable to avoid allocate new memory every time
        cv::cvtColor(rgba_output, rgb_output, cv::COLOR_RGBA2RGB);

        // Copy depth to output
        //
        // Reference: Azure-Kinect-Samples/pipe-to-python-samples/main.cpp
        std::memcpy( reinterpret_cast<uint8_t*> xyz_output.data,
                     reinterpret_cast<uint8_t*> k4a_image_get_buffer( point_cloud_transformed_image_coordinate ), 
                     k4a_image_get_size( point_cloud_transformed_image_coordinate ) );

        // Free resources
    ReleaseRunTimeResource:
        if ( image_sample ) k4a_image_release( image_sample );
        if ( depth_sample ) k4a_image_release( depth_sample );
        if ( ir_sample ) k4a_image_release( ir_sample );
        if ( depth_transformed_image_coordinate ) k4a_image_release(depth_transformed_image_coordinate);
        // Now we're using cv::Mat buffer for k4a_image_t. This memory should be free by caller
        // k4a_image_release(point_cloud_transformed_image_coordinate);
        k4a_capture_release(capture);
    }

    // ( cx, cy, fx, fy )
    cv::Vec4d AzureKinectCamera::getColorIntrinsics() const
    {
        return intrinsic;
    }

    const std::string AzureKinectCamera::getModelName() const
    {
        return std::string{"AzureKinect"};
    }

    // (width, height)
    cv::Size AzureKinectCamera::getSize() const
    {
        return cv::Size{ img_width, img_height };
    }

    int getHeight() const
    {
        return img_height;
    }

    int getWidth() const
    {
        return img_width;
    }

}