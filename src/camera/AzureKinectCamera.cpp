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
 */

#include <k4a/k4a.h> // AzureKinect API
#include "openark/camera/AzureKinectCamera.h"

namespace ark
{
    // Delegate constructor call
    AzureKinectCamera::AzureKinectCamera(): \
        device(NULL), capture(NULL), calibration(NULL), transformation(NULL), \
        TIMEOUT_IN_MS(1000) {};

    AzureKinectCamera::~AzureKinectCamera()
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
            // TODO : change print error to print to google test
            printf("No K4A devices found\n");
            goto Exit;
        }

        // Open camera
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
        {
            // TODO : change print error to print to google test
            printf("Failed to open device\n");
            goto Exit;
        }

        // Camera configruation
        // TODO add depth on & depth off support
        // Configure a stream of 4096x3072 BRGA color data at 15 frames per second
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        // currently only fixed frame rate is supported
        config.camera_fps       = K4A_FRAMES_PER_SECOND_30;
        // RGBA fp32
        //config.color_format     = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        // GRAY uint8
        config.color_format     = K4A_IMAGE_FORMAT_CUSTOM8;
        // trun off  image
        config.color_format     = K4A_COLOR_RESOLUTION_OFF;
        // GRAY uint16
        //config.color_format     = K4A_IMAGE_FORMAT_CUSTOM16;
        // currently only fixed ratio 1280 * 720  16:9 is supported
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        // currently only fixed ratio 640x576 is supported
        // NOTICE: depth size is not same as RGB size
        config.depth_mode       = K4A_DEPTH_MODE_NFOV_UNBINNED;
        // trun off depth
        //config.depth_mode       = K4A_DEPTH_MODE_OFF;
        // ensures that depth and color images are both available in the capture
        // objects may be produced only a single image when the corresponding image is dropped.
        config.synchronized_images_only = true; 

        // Get calibration information
        if (K4A_RESULT_SUCCEEDED != \
            k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
        {
            // TODO : change print error to print to google test
            printf("Failed to get calibration\n");
            goto Exit;
        }

        // Set Intrinsic matrix
        // Azure-Kinect-Sensor-SDK/examples/calibration/main.cpp
        auto& color_camera_intrin = calibration.color_camera_calibration.intrinsics.parameters.param;
        intrinsic.at(0) = color_camera_intrin.cx;
        intrinsic.at(1) = color_camera_intrin.cy;
        intrinsic.at(2) = color_camera_intrin.fx;
        intrinsic.at(3) = color_camera_intrin.fy;

        // Set image resolution 
        image_resolution_width = calibration.color_camera_calibration.resolution_width;
        image_resolution_height = calibration.color_camera_calibration.resolution_height;

        // Set transformation
        transformation = k4a_transformation_create(&calibration);
    
        // Start camera
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
        {
            // TODO : change print error to print to google test
            printf("Failed to start device\n");
            goto Exit;
        }

        // Start IMU
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
        {
            // TODO : change print error to print to google test
            printf("Failed to start imu\n");
            goto Exit;
        }

    Exit:
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }


    void AzureKinectCamera::getCurrData(MultiCameraFrame::Ptr framePtr)
    {
        k4a_image_t image_sample, depth_sample;
        k4a_imu_sample_t imu_sample;

        // Capture depth & image data
        //
        // Azure-Kinect-Sensor-SDK/examples/streaming/main.c
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            // TODO change continue
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            // TODO change exit
            goto Exit;
        }

        // Extract BGRA/GRAY from capture
        image_sample = k4a_capture_get_color_image(capture);
        if ( !image_sample )
        {
            printf("Failed to read image from capture\n");
            // TODO add return condition
        }

        // Extract depth from capture
        depth_sample = k4a_capture_get_depth_image(capture);
        if ( !depth_sample )
        {
            printf("Failed to read depth from capture\n");
            // TODO add return condition
        }

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
                continue;
                // TODO change below code to jump out of while loop
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a imu sample\n");
                // TODO change below code to jump out of while loop
                goto Exit;
            }
            // TODO first resize container to 200 / 30
            // TODO add imu datas to container
        }

        // Transform depth to color coordinate
        // https://github.com/Microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/transformation
        // https://docs.microsoft.com/en-us/azure/kinect-dk/use-image-transformation
        // Allocate memory for transformed depth image
        int image_width_pixels = k4a_image_get_width_pixels(image_sample);
        int image_height_pixels = k4a_image_get_height_pixels(image_sample);
        k4a_image_t depth_transformed_image_coordinate = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                    image_width_pixels,
                                                    image_height_pixels,
                                                    image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                    &depth_transformed_image_coordinate))
        {
            printf("Failed to create transformed depth image\n");
            // TODO handle this
            goto Exit;
        }        

        // Transform depth
        if (K4A_RESULT_SUCCEEDED != \
            k4a_transformation_depth_image_to_color_camera(transformation, \
                                                           depth_sample, \
                                                           depth_transformed_image_coordinate ))
        {
            printf("Failed to compute transformed depth image\n");
            return false;
        }

        // Free resources
        // TODO find a way to avoid free resources every time
        k4a_image_release(depth_transformed_image_coordinate);
        k4a_image_release(image_sample);
        k4a_image_release(depth_sample);
        k4a_capture_release(capture);
    }

    // ( cx, cy, fx, fy )
    cv::Vec4d AzureKinectCamera::getColorIntrinsics()
    {
        return intrinsic;
    }

    std::string AzureKinectCamera::getModelName() const
    {
        return std::string{"AzureKinect"};
    }

    // (width, height)
    cv::Size AzureKinectCamera::getImageSize() const
    {
        return cv::Size{ image_resolution_width, image_resolution_height };
    }

    int getHeight() const
    {
        return image_resolution_height;
    }

    int getWidth() const
    {
        return image_resolution_width;
    }

}