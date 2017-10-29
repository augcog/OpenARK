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
    sm->Release();
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
    vector<cv::Point3f>  xyzBuffer;

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

    auto depthMap = sample->depth;
    Intel::RealSense::Image::ImageData depthImage;

    depthMap->AcquireAccess(Intel::RealSense::Image::ACCESS_READ, &depthImage);

    cv::Mat img;
    Converter::ConvertPXCImageToOpenCVMat(depthMap, depthImage, &img);

    cv::imshow("Depth Image by OpenARK", Visualizer::visualizeDepthMap(img));

    auto imgInfo = depthMap->QueryInfo();
    depth_width = imgInfo.width;
    depth_height = imgInfo.height;

    int num_pixels = depth_width * depth_height;
    auto projection = device->CreateProjection();
    auto pos3D = new Intel::RealSense::Point3DF32[num_pixels];

    sts = projection->QueryVertices(depthMap, &pos3D[0]);
    depthMap->ReleaseAccess(&depthImage);

    if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
    {
        wprintf_s(L"Projection was unsuccessful! \n");
        sm->Close();
    }

    projection->Release();

    xyzBuffer.clear();

    for (auto k = 0; k < num_pixels; k++)
    {
        xyzBuffer.emplace_back(cv::Point3f(pos3D[k].x / 1000.0f, pos3D[k].y / 1000.0f, pos3D[k].z / 1000.0f));
    }

    delete[] pos3D;

    xyzMap = cv::Mat(xyzBuffer, true).reshape(3, 480);
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