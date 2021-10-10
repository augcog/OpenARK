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


#include "AzureKinectCamera.h"

namespace ark
{
    // Delegate constructor call
    AzureKinectCamera::AzureKinectCamera() : AzureKinectCamera( camParam ) {}

    AzureKinectCamera::AzureKinectCamera( const CameraParameter& camParam ) 
    : camParam( camParam )
    {
        // Detail camera setup mostly follow
        // https://docs.microsoft.com/en-us/azure/kinect-dk/build-first-app

    }

    AzureKinectCamera::~AzureKinectCamera()
    {

    }

    void AzureKinectCamera::start()
    {

    }


    void AzureKinectCamera::update(MultiCameraFrame::Ptr framePtr)
    {

    }

    std::vector<float> AzureKinectCamera::getColorIntrinsics()
    {

    }

    std::string AzureKinectCamera::getModelName() const
    {
        return std::string{"AzureKinect"};
    }

    cv::Size AzureKinectCamera::getImageSize() const
    {
        return cv::Size{ camParam.width, camParam.height };
    }

}