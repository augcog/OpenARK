#pragma once
// OpenCV Libraries
#include "stdafx.h"
#include "version.h"

// OpenARK Libraries
#include "DepthCamera.h"
#include "Converter.h"

namespace ark {
    /**
    * Class defining the behavior of an SR300 Camera.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class SR300Camera : public DepthCamera
    {
    public:

        /**
        * Public constructor initializing the SR300 Camera.
        * @param use_live_sensor uses input from real sensor if TRUE. Otherwise reads from input file. Default is set to TRUE.
        */
        explicit SR300Camera(bool use_live_sensor = true);

        /**
        * Deconstructor for the SR300 Camera.
        */
        ~SR300Camera();

        /**
        * Gets new frame from sensor.
        * Updates xyzMap, ampMap, and flagMap. Resets clusters.
        */
        void update() override;

        /**
         * Returns true if an RGB image is available from this camera.
         * @return true if an RGB image is available from this camera.
         */
        bool hasRGBImage() const;

        /**
         * Returns true if an infrared (IR) image is available from this camera.
         * @return true if an infrared (IR) image is available from this camera.
         */
        bool hasIRImage() const;

        /**
        * Gracefully closes the SR300 camera.
        */
        void destroyInstance();

    private:
        /**
        * Getter method for the x-coordinate at (i,j).
        * @param i ith row
        * @param j jth column
        * @return x-coodinate at (i,j)
        */
        float getX(int i, int j) const;

        /**
        * Getter method for the x-coordinate at (i,j).
        * @param i ith row
        * @param j jth column
        * @return x-coodinate at (i,j)
        */
        float getY(int i, int j) const;

        /**
        * Getter method for the x-coordinate at (i,j).
        * @param i ith row
        * @param j jth column
        * @return x-coodinate at (i,j)
        */
        float getZ(int i, int j) const;

        /**
        * Update the z-coordinates of the xyzMap.
        */
        void fillInZCoords();

        /**
        * Update the values in the ampMap.
        */
        void fillInAmps();

        /**
         * Initializat the camera
         */
        void initCamera();

        //Private Variables
        float* dists;
        float* amps;
        cv::Mat frame;
        const int depth_fps = 30;
        int depth_width;
        int depth_height;
        cv::Size bufferSize;
        const Intel::RealSense::Sample *sample;
        Intel::RealSense::SenseManager *sm = Intel::RealSense::SenseManager::CreateInstance();
        Intel::RealSense::Session *session = sm->QuerySession();
        Intel::RealSense::Device *device;
        Intel::RealSense::CaptureManager *cm;

    };
}