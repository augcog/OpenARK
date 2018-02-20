#pragma once

#include <stdafx.h>

// PMD Libraries
#include <pmdsdk2.h>

// OpenARK Libraries
#include "DepthCamera.h"
namespace ark {

    /**
    * Class defining the behavior of a PMD Camera.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class PMDCamera : public DepthCamera
    {
    public:
        /**
        * Public constructor initializing the PMD Camera.
        * @param use_live_sensor uses input from real sensor if TRUE. Otherwise reads from input file. Default is set to TRUE.
        */
        explicit PMDCamera(bool use_live_sensor = true);

        /**
        * Destructor for the PMD Camera.
        */
        ~PMDCamera();

        /**
        * Gets new frame from sensor.
        * Updates xyzMap, ampMap, and flagMap. Resets clusters.
        */
        void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                             cv::Mat & amp_map, cv::Mat & flag_map) override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /** get camera frame width */
        int getWidth() const override;

        /** get camera frame height */
        int getHeight() const override;

        float flagMapConfidenceThreshold() const override;

        int ampMapInvalidFlagValue() const override;

        /**
         * Returns true if a flag map is available from this camera.
         */
        bool hasAmpMap() const override;

        /**
         * Returns true if a flag map is available from this camera.
         */
        bool hasFlagMap() const override;

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

        // Private Variables
        const char* SOURCE_PLUGIN = "camboardpico";
        const char* SOURCE_PARAM = "";
        const char* PROC_PLUGIN = "camboardpicoproc";
        const char* PROC_PARAM = "";
        PMDHandle hnd;
        PMDDataDescription dd;
        char err[128]; // Char array for storing PMD's error log
        int numPixels;
        float* dists;
        float* amps;
        cv::Mat frame;
    };

}
    /*
    * \include SensorIO.cpp
    * Example of how to read from sensor
    */