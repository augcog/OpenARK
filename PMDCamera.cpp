#include "stdafx.h"
#include "PMDCamera.h"

namespace ark {
    /***
    Private constructor for the PMD depth sensor
    ***/
    PMDCamera::PMDCamera(bool use_live_sensor)
    {
        if (!use_live_sensor)
        {
            return;
        }

        std::cout << "Trying to open pmd\n";
        auto res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM); //Open the PMD sensor

        if (res != PMD_OK)
        {
            pmdGetLastError(0, err, 128);
            fprintf(stderr, "Could not connect: %s\n", err);
            return;
        }

        printf("opened sensor\n");

        // Updating the sensor is necessary before any data can be retrieved
        res = pmdUpdate(hnd);

        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Couldn't transfer data: %s\n", err);
            pmdClose(hnd);
            return;
        }

        printf("acquired image\n");

        // res: Structure which contains various meta-information about the data delivered by your Nano.
        // It is advisabe to always use the data delivered by this struct (for example the width and height of the imager
        // and the image format). Please refer to the PMSDSK documentation for more information
        res = pmdGetSourceDataDescription(hnd, &dd);

        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Couldn't get data description: %s\n", err);
            pmdClose(hnd);
            return;
        }

        printf("retrieved source data description\n");

        if (dd.subHeaderType != PMD_IMAGE_DATA)
        {
            fprintf(stderr, "Source data is not an image!\n");
            pmdClose(hnd);
            return;
        }

        numPixels = dd.img.numRows * dd.img.numColumns; // Number of pixels in camera
        dists = new float[3 * numPixels]; // Dists contains XYZ values. needs to be 3x the size of numPixels
        amps = new float[numPixels];
        frame.create(dd.img.numRows, dd.img.numColumns, CV_8UC3);
    }
    
    const std::string PMDCamera::getModelName() const {
        return "PMD";
    }

    int PMDCamera::getWidth() const {
        return 176;
    }

    int PMDCamera::getHeight() const {
        return 120;
    }

    float PMDCamera::flagMapConfidenceThreshold() const {
        return (60.0f / 255.0f*500.0f);
    }

    int PMDCamera::ampMapInvalidFlagValue() const {
        return PMD_FLAG_INVALID;
    }

    bool PMDCamera::hasAmpMap() const
    {
        return true;
    }

    bool PMDCamera::hasFlagMap() const
    {
        return true;
    }

    /***
    Public deconstructor for the PMD depth sensor
    ***/
    PMDCamera::~PMDCamera()
    {
        printf("closing sensor\n");
        pmdClose(hnd);
        printf("sensor closed\n");
    }

    /***
    Create xyzMap, zMap, ampMap, and flagMap from sensor input
    ***/
    void PMDCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, cv::Mat & fisheye_map,
                             cv::Mat & amp_map, cv::Mat & flag_map) 
    {
        // fill in amp map
        auto res = pmdGetAmplitudes(hnd, amps, numPixels * sizeof(float));

        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Couldn't get amplitudes: %s\n", err);
            pmdClose(hnd);
            return;
        }

        amp_map.data = reinterpret_cast<uchar *>(amps);

        // fill in Z coordinates
        auto res = pmdGet3DCoordinates(hnd, dists, 3 * numPixels * sizeof(float)); //store x,y,z coordinates dists (type: float*)
        //float * zCoords = new float[1]; //store z-Coordinates of dists in zCoords

        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Couldn't get 3D coordinates: %s\n", err);
            pmdClose(hnd);
            return;
        }

        xyz_map = cv::Mat(xyzMap.size(), xyzMap.type(), dists);

        // Flags. Helps with denoising.
        auto flags = new unsigned[ampMap.cols*ampMap.rows];
        auto res = pmdGetFlags(hnd, flags, numPixels * sizeof(unsigned));

        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Couldn't get the flags: %s\n", err);
            pmdClose(hnd);
            return;
        }

        flag_map.data = reinterpret_cast<uchar *>(flags);

        res = pmdUpdate(hnd);
        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Couldn't update the PMD camera: %s\n", err);
            pmdClose(hnd);
            return;
        }

        delete flags;
    }


    /***
    Returns the X value at (i, j)
    ***/
    float PMDCamera::getX(int i, int j) const
    {
        int flat = j * dd.img.numColumns * 3 + i * 3;
        return dists[flat];
    }

    /***
    Returns the Y value at (i, j)
    ***/
    float PMDCamera::getY(int i, int j) const
    {
        int flat = j * dd.img.numColumns * 3 + i * 3;
        return dists[flat + 1];
    }

    /***
    Returns the Z value at (i, j)
    ***/
    float PMDCamera::getZ(int i, int j) const
    {
        int flat = j * dd.img.numColumns * 3 + i * 3;
        return dists[flat + 2];
    }
}