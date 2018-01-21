#include "stdafx.h"
#include "version.h"
#include "SR300Camera.h"
#include "Visualizer.h"

using namespace Intel::RealSense;

namespace ark {
    /***
    Private constructor for the Intel RealSense SR300 camera depth sensor
    ***/
    SR300Camera::SR300Camera(bool use_live_sensor) : dists(nullptr), amps(nullptr), depth_width(0), depth_height(0), sample(nullptr)
    {
        session->SetCoordinateSystem(CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);

        if (!sm)
        {
            wprintf_s(L"Unable to create the SenseManager!\n");
            return;
        }

        initCamera();
    }

    /***
    Destructor for the SR300 Camera depth sensor
    ***/
    SR300Camera::~SR300Camera() {
        printf("closing sensor\n");
        sm->Close();
        printf("sensor closed\n");
    };

    // overrided model name
    const std::string SR300Camera::getModelName() const
    {
        return "SR300";
    }

    // overrided width
    int SR300Camera::getWidth() const {
        // cut off 35 px to eliminate shadow on right
        return REAL_WID - 35;
    }

    // overrided height
    int SR300Camera::getHeight() const {
        return REAL_HI;
    }

    /**
    * true if has RGB image (override)
    */
    bool SR300Camera::hasRGBMap() const {
        // not used
        return false;
    }

    /**
    * true if has IR image (override)
    */
    bool SR300Camera::hasIRMap() const {
        return true;
    }

    /**
    * Create xyzMap, zMap, ampMap, and flagMap from sensor input (override)
    * @param [out] xyz_map XYZ map (projection point cloud). CV_32FC3
    * @param [out] rgb_map RGB image. CV_8UC3 (NOT USED)
    * @param [out] ir_map IR image. CV_8UC1
    * @param [out] amp_map amplitude map. CV_32FC1 (NOT USED)
    * @param [out] flag_map flag map. CV_8UC1 (NOT USED)
    */
    void SR300Camera::update(MatPtr & xyz_map, MatPtr & rgb_map, MatPtr & ir_map, 
                            MatPtr & amp_map, MatPtr & flag_map) 
    {
        Status sts = sm->AcquireFrame(true);

        if (sts < STATUS_NO_ERROR)
        {
            if (sts == Status::STATUS_STREAM_CONFIG_CHANGED)
            {
                wprintf_s(L"Stream configuration was changed, re-initializing\n");
                sm->ReleaseFrame();
                sm->Close();
                badInputFlag = true;
                return;
            }
        }

        sample = sm->QuerySample();

        if (!sample || sample->depth == nullptr) {
            wprintf_s(L"Couldn't connect to camera, retrying in 0.5s...\n");
            sm->ReleaseFrame(); sm->Close(); 
            badInputFlag = true;
            boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
            initCamera();
            return;
        }

        badInputFlag = false;

        // get images from camera
        Image * depthSource = sample->depth, *irSource = sample->ir;
        Image::ImageData depthImage, irMap;
        depthSource->AcquireAccess(Image::ACCESS_READ, Image::PixelFormat::PIXEL_FORMAT_DEPTH_F32, &depthImage);
        irSource->AcquireAccess(Image::ACCESS_READ, Image::PixelFormat::PIXEL_FORMAT_Y8, &irMap);

        // create projection from depth image
        ImageInfo imgInfo = depthSource->QueryInfo();
        depth_width = imgInfo.width;
        depth_height = imgInfo.height;

        int num_pixels = depth_width * depth_height;
        Projection * projection = device->CreateProjection();
        Point3DF32 * pos3D = new Point3DF32[num_pixels];

        sts = projection->QueryVertices(depthSource, &pos3D[0]);

        if (sts < Status::STATUS_NO_ERROR)
        {
            wprintf_s(L"Projection was unsuccessful! \n");
            sm->ReleaseFrame();
            sm->Close();
            return;
        }

        projection->Release();

        // convert IR image
        cv::Mat irTmp;
        Converter::ConvertPXCImageToOpenCVMat(irSource, irMap, &irTmp);
        *ir_map = irTmp(cv::Rect(0, 0, getWidth(), getHeight()));

        // release access
        depthSource->ReleaseAccess(&depthImage);
        irSource->ReleaseAccess(&irMap);

        // convert point cloud
        int pixels_per_row = num_pixels / getHeight(), k = 0, wid = getWidth();
        for (int r = 0; r < getHeight(); ++r)
        {
            Vec3f * ptr = xyz_map->ptr<Vec3f>(r);
            for (int c = 0; c < pixels_per_row; ++c) {
                if (c < wid) {
                    ptr[c] = Vec3f(pos3D[k].x , pos3D[k].y, pos3D[k].z) / 1000.0f;
                }
                ++k;
            }
        }

        // clean up
        delete[] pos3D;
        sm->ReleaseFrame();
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

    // Initialize camera (helper)
    void SR300Camera::initCamera() {
        cm = sm->QueryCaptureManager();
        auto sts = Status::STATUS_DATA_UNAVAILABLE;

        sm->EnableStream(Capture::STREAM_TYPE_DEPTH, REAL_WID, REAL_HI, depth_fps);
        sm->EnableStream(Capture::STREAM_TYPE_IR, REAL_WID, REAL_HI, depth_fps);

        sts = sm->Init();
        device = cm->QueryDevice();
    }


};
