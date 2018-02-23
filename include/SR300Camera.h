#pragma once
// OpenCV Libraries
#include "version.h"
#include <opencv2/core.hpp>

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
        * Destructor for the SR300 Camera.
        */
        ~SR300Camera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /** 
         * Returns the width of the SR300 camera frame 
         */
        int getWidth() const override;

        /** 
         * Returns the height of the SR300 camera frame 
         */
        int getHeight() const override;

        /**
         * Returns true if an RGB image is available from this camera.
         * @return true if an RGB image is available from this camera.
         */
        bool hasRGBMap() const override;

        /**
         * Returns true if an infrared (IR) image is available from this camera.
         * @return true if an infrared (IR) image is available from this camera.
         */
        bool hasIRMap() const override;

        /** Shared pointer to SR300 camera instance */
        typedef std::shared_ptr<SR300Camera> Ptr;

    protected:
        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                            cv::Mat & amp_map, cv::Mat & flag_map) override;

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
         * Initialize the camera, opening channels and resetting to initial configurations
         */
        void initCamera();

        // Private Variables
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

        // actual width of the SR300 camera depth frame
        static const int REAL_WID = 640;

        // actual height of the SR300 camera depth frame
        static const int REAL_HI = 480;
    };
}