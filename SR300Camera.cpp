#include "stdafx.h"
#include "version.h"
#include "SR300Camera.h"
#include "Visualizer.h"

using namespace Intel::RealSense;

namespace ark {
    static const int REAL_WID = 640, REAL_HI = 480;

    /***
    Private constructor for the Intel RealSense SR300 camera depth sensor
    ***/
    SR300Camera::SR300Camera(bool use_live_sensor) : dists(nullptr), amps(nullptr), depth_width(0), depth_height(0), sample(nullptr)
    {
        session->SetCoordinateSystem(CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);

        // cut off 35 px to eliminate shadow on right
        X_DIMENSION = REAL_WID - 35;
        Y_DIMENSION = REAL_HI;

        if (!sm)
        {
            wprintf_s(L"Unable to create the SenseManager!\n");
            return;
        }

        initCamera();
    }

    // Initialize camera
    void SR300Camera::initCamera() {
        cm = sm->QueryCaptureManager();
        auto sts = Status::STATUS_DATA_UNAVAILABLE;

        sm->EnableStream(Capture::STREAM_TYPE_DEPTH, REAL_WID, REAL_HI, depth_fps);
        sm->EnableStream(Capture::STREAM_TYPE_IR, REAL_WID, REAL_HI, depth_fps);

        sts = sm->Init();
        device = cm->QueryDevice();
    }

    /***
    Public deconstructor for the SR300 Camera depth sensor
    ***/
    SR300Camera::~SR300Camera() {};

    void SR300Camera::destroyInstance()
    {
        _badInput = true;
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
        Status sts = sm->AcquireFrame(true);
        if (sts < STATUS_NO_ERROR)
        {
            if (sts == Status::STATUS_STREAM_CONFIG_CHANGED)
            {
                wprintf_s(L"Stream configuration was changed, re-initializing\n");
                sm->Close();
                _badInput = true;
            }
        }

        sample = sm->QuerySample();

        if (!sample || sample->depth == nullptr) {
            wprintf_s(L"Couldn't connect to camera, retrying in 0.5s...\n");
            sm->ReleaseFrame();
            sm->Close();
            _badInput = true;
            cv::waitKey(500);
            initCamera();
            return;
        }
        _badInput = false;

        Image * depthSource = sample->depth, *irSource = sample->ir;

        Image::ImageData depthImage, irImage;

        depthSource->AcquireAccess(Image::ACCESS_READ, Image::PixelFormat::PIXEL_FORMAT_DEPTH_F32, &depthImage);

        ImageInfo imgInfo = depthSource->QueryInfo();
        depth_width = imgInfo.width;
        depth_height = imgInfo.height;

        int num_pixels = depth_width * depth_height;
        Projection * projection = device->CreateProjection();
        Point3DF32 * pos3D = new Point3DF32[num_pixels];

        sts = projection->QueryVertices(depthSource, &pos3D[0]);
        depthSource->ReleaseAccess(&depthImage);

        if (sts < Status::STATUS_NO_ERROR)
        {
            wprintf_s(L"Projection was unsuccessful! \n");
            sm->Close();
        }

        projection->Release();

        int pixels_per_row = num_pixels / Y_DIMENSION;
        xyzMap = cv::Mat(Y_DIMENSION, pixels_per_row, CV_32FC3);

        int k = 0;
        for (int r = 0; r < Y_DIMENSION; ++r)
        {
            Vec3f *ptr = xyzMap.ptr<Vec3f>(r);
            for (int c = 0; c < pixels_per_row; ++c) {
                ptr[c] = Vec3f(pos3D[k].x / 1000.0f, pos3D[k].y / 1000.0f, pos3D[k].z / 1000.0f);
                ++k;
            }
        }

        // convert IR image
        irSource->AcquireAccess(Image::ACCESS_READ, Image::PixelFormat::PIXEL_FORMAT_Y16, &irImage);

        Converter::ConvertPXCImageToOpenCVMat(irSource, irImage, &this->irImage);

        irSource->ReleaseAccess(&irImage);

        cv::Rect rect = cv::Rect(0, 0, X_DIMENSION, Y_DIMENSION);

        // crop images to correct dimensions
        if (this->irImage.rows != Y_DIMENSION || this->irImage.cols != X_DIMENSION) {
            this->irImage = this->irImage(rect);
        }

        if (xyzMap.rows != Y_DIMENSION || xyzMap.cols != X_DIMENSION) {
            xyzMap = xyzMap(rect);
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

    /**
    * Returns the X value at (i, j)
    */
    float SR300Camera::getX(int i, int j) const
    {
        int flat = j * depth_width * 3 + i * 3;
        return dists[flat];
    }

    /**
    * Returns the Y value at (i, j)
    */
    float SR300Camera::getY(int i, int j) const
    {
        int flat = j * depth_width * 3 + i * 3;
        return dists[flat + 1];
    }

    /**
    * Returns the Z value at (i, j)
    */
    float SR300Camera::getZ(int i, int j) const
    {
        auto flat = j * depth_width * 3 + i * 3;
        return dists[flat + 2];
    }

    /*
    * True if has RGB image
    */
    bool SR300Camera::hasRGBImage() const {
        // disabled for now
        return false;
    }

    /*
    * True if has IR image
    */
    bool SR300Camera::hasIRImage() const {
        return irImage.rows > 0;
    }
};
