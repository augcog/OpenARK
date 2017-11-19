#include "stdafx.h"
#include "SR300Camera.h"
#include "Visualizer.h"

using namespace std;


/***
Private constructor for the Intel RealSense SR300 camera depth sensor
***/
SR300Camera::SR300Camera(bool use_live_sensor): dists(nullptr), amps(nullptr), depth_width(0), depth_height(0), sample(nullptr)
{
    session->SetCoordinateSystem(Intel::RealSense::CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);
    X_DIMENSION = 640;
    Y_DIMENSION = 480;
    if (!sm)
    {
        wprintf_s(L"Unable to create the SenseManager\n");
    }
    cm = sm->QueryCaptureManager();
    auto sts = Intel::RealSense::Status::STATUS_DATA_UNAVAILABLE;

    sm->EnableStream(Intel::RealSense::Capture::StreamType::STREAM_TYPE_DEPTH, X_DIMENSION, Y_DIMENSION, depth_fps);
    sts = sm->Init();
    if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
    {
        sm->Close();
        sm->EnableStream(Intel::RealSense::Capture::STREAM_TYPE_DEPTH);
        sts = sm->Init();
        if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
        {
            sm->Close();
            sts = sm->Init();
        }
    }
    device = cm->QueryDevice();
}

/***
Public deconstructor for the SR300 Camera depth sensor
***/
SR300Camera::~SR300Camera() {};

void SR300Camera::destroyInstance()
{
    printf("closing sensor\n");
    //sm->Release();
    sm->Close();
    printf("sensor closed\n");
}

/***
Create xyzMap, zMap, ampMap, and flagMap from sensor input
***/
void SR300Camera::update()
{
    initializeImages();
    fillInAmps();
    fillInZCoords();
    sm->ReleaseFrame();
}

/***
Reads the depth data from the sensor and fills in the matrix
***/
void SR300Camera::fillInZCoords()
{
    int res;
    auto sts = sm ->AcquireFrame(true);
    if (sts < Intel::RealSense::STATUS_NO_ERROR)
    {
        if (sts == Intel::RealSense::Status::STATUS_STREAM_CONFIG_CHANGED)
        {
            wprintf_s(L"Stream configuration was changed, re-initializing\n");
            sm ->Close();
        }
    }

    sample = sm->QuerySample();

    Intel::RealSense::Image * depthMap = sample->depth;

    if (depthMap == nullptr) {
        wprintf_s(L"Couldn't connect to camera. Ctrl+C to exit.\n");
        sm ->Close();
        return;
    }

    Intel::RealSense::Image::ImageData depthImage;

    depthMap->AcquireAccess(Intel::RealSense::Image::ACCESS_READ, &depthImage);

    cv::Mat img;
    Converter::ConvertPXCImageToOpenCVMat(depthMap, depthImage, &img);

    #ifdef DEMO
    cv::imshow("OpenARK Depth Image", Visualizer::visualizeDepthMap(img));
    #endif

    Intel::RealSense::ImageInfo imgInfo = depthMap->QueryInfo();
    depth_width = imgInfo.width;
    depth_height = imgInfo.height;

    int num_pixels = depth_width * depth_height;
    Intel::RealSense::Projection * projection = device->CreateProjection();
    Intel::RealSense::Point3DF32 * pos3D = new Intel::RealSense::Point3DF32[num_pixels];

    sts = projection->QueryVertices(depthMap, &pos3D[0]);
    depthMap->ReleaseAccess(&depthImage);

    if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
    {
        wprintf_s(L"Projection was unsuccessful! \n");
        sm->Close();
    }

    projection->Release();

    const int NUM_ROWS = 480;
    int pixels_per_row = num_pixels / NUM_ROWS;
    xyzMap = cv::Mat(NUM_ROWS, pixels_per_row, CV_32FC3);

    int k = 0;
    for (int r = 0; r < NUM_ROWS; ++r)
    {
        cv::Vec3f *ptr = xyzMap.ptr<cv::Vec3f>(r);
        for (int c = 0; c < pixels_per_row; ++c) {
            ptr[c] = cv::Vec3f(pos3D[k].x / 1000.0f, pos3D[k].y / 1000.0f, pos3D[k].z / 1000.0f);
            ++k;
        }
    }

    delete[] pos3D;
}

/***
Reads the amplitude data from the sensor and fills in the matrix
***/
void SR300Camera::fillInAmps()
{
    ampMap.data = nullptr;
}

/***
Returns the X value at (i, j)
***/
float SR300Camera::getX(int i, int j) const
{
    auto flat = j * depth_width * 3 + i * 3;
    return dists[flat];
}

/***
Returns the Y value at (i, j)
***/
float SR300Camera::getY(int i, int j) const
{
    auto flat = j * depth_width * 3 + i * 3;
    return dists[flat + 1];
}

/***
Returns the Z value at (i, j)
***/
float SR300Camera::getZ(int i, int j) const
{
    auto flat = j * depth_width * 3 + i * 3;
    return dists[flat + 2];
}