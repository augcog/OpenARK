#include "stdafx.h"
#include "version.h"
#include "SR300Camera.h"
#include "Visualizer.h"

using namespace std;
using namespace Intel::RealSense;

/***
Private constructor for the Intel RealSense SR300 camera depth sensor
***/
SR300Camera::SR300Camera(bool use_live_sensor): dists(nullptr), amps(nullptr), depth_width(0), depth_height(0), sample(nullptr)
{
    session->SetCoordinateSystem(CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);
    X_DIMENSION = 640;
    Y_DIMENSION = 480;
    if (!sm)
    {
        wprintf_s(L"Unable to create the SenseManager\n");
    }
    cm = sm->QueryCaptureManager();
    auto sts = Status::STATUS_DATA_UNAVAILABLE;

    sm->EnableStream(Capture::STREAM_TYPE_DEPTH, X_DIMENSION, Y_DIMENSION, depth_fps);
    sm->EnableStream(Capture::STREAM_TYPE_IR, X_DIMENSION, Y_DIMENSION, depth_fps);

    sts = sm->Init();
    if (sts < Status::STATUS_NO_ERROR)
    {
        sm->Close();
        sm->EnableStream(Capture::STREAM_TYPE_DEPTH);
        sts = sm->Init();
        if (sts < Status::STATUS_NO_ERROR)
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
    Status sts = sm ->AcquireFrame(true);
    if (sts < STATUS_NO_ERROR)
    {
        if (sts == Status::STATUS_STREAM_CONFIG_CHANGED)
        {
            wprintf_s(L"Stream configuration was changed, re-initializing\n");
            sm ->Close();
        }
    }

    sample = sm->QuerySample();

    if (!sample || sample->depth == nullptr) {
        wprintf_s(L"Couldn't connect to camera.\n");
        sm->Close();
        return;
    }

    Image * depthSource = sample->depth, * irSource = sample->ir;

    Image::ImageData depthImage , irImage;

    depthSource->AcquireAccess(Image::ACCESS_READ, &depthImage);
    irSource->AcquireAccess(Image::ACCESS_READ, Image::PixelFormat::PIXEL_FORMAT_Y16, &irImage);

    cv::Mat depthImageCV, irImageCV;
    Converter::ConvertPXCImageToOpenCVMat(depthSource, depthImage, &depthImageCV);
    Converter::ConvertPXCImageToOpenCVMat(irSource, irImage, &irImageCV);

    this->irImage = irImageCV;

    ImageInfo imgInfo = depthSource->QueryInfo();
    depth_width = imgInfo.width;
    depth_height = imgInfo.height;

    int num_pixels = depth_width * depth_height;
    Projection * projection = device->CreateProjection();
    Point3DF32 * pos3D = new Point3DF32[num_pixels];

    sts = projection->QueryVertices(depthSource, &pos3D[0]);
    depthSource->ReleaseAccess(&depthImage);
    irSource->ReleaseAccess(&irImage);

    if (sts < Status::STATUS_NO_ERROR)
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

bool SR300Camera::hasRGBImage() const {
    // we do have access to the RGB image, but since it's not used right now we'll disable it
    return false;
}

bool SR300Camera::hasIRImage() const {
    return true;
}
