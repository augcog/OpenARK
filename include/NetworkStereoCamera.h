#pragma once
// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>

// OpenARK Libraries
#include "StereoCamera.h"
#include "Converter.h"
#include "NetworkLib.h"

#include <vector>

namespace ark {
    /**
    * Class characterizing a generic depth-from-stereo system
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class NetworkStereoCamera : public StereoCamera
    {
    public:
        /** Represents a synced stereo frame received from the network */
        struct Frame {
            /** Image data */
            cv::Mat xyzMap, leftImage;

            /** ID of client that sent the image */
            uint32_t clientID;

            /** time the frame was taken */
            size_t timestamp;

            typedef std::shared_ptr<Frame> Ptr;
        };

        /** Represents an interaction callback function */
        typedef std::function<void(const Frame)> StereoUpdateCallback;

        /**
        * Public constructor initializing the stereo camera.
        * @param calib camera calibration information
        * @param port UDP port to use
        * @param stereoUpdateCallback a function to call when the camera finishes processing a frame sent by a client
        * @param sgbmConf optionally, custom SGBM configuration
        */
        explicit NetworkStereoCamera(StereoCalibration::Ptr calib, 
            int port = 23333, StereoUpdateCallback stereoUpdateCallback = nullptr, SGBMConfig::Ptr sgbmConf = nullptr);

        //~NetworkStereoCamera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;
        
        // network stereo specific functions

        /** Returns true if a frame is ready (getLatestFrame guarenteed to return some frame) */
        bool frameReady() const;

        /** Get latest frame, containing the computed xyzMap (32FC3), left gray image (8U), and the client ID */
        Frame getLatestFrame() const;

        /** Set function to call when the camera finsishes processing a frame  */
        void setStereoUpdateCallback(StereoUpdateCallback callback);

        // client communication

        /** Get number of clients connected through this camera class */
        int numClients() const;

        /** Get ID of 'index'th client */
        uint32_t getClientByIndex(int index) const;

        /** Get all IDs for current clients */
        std::vector<uint32_t> getClients(uint32_t client) const;

        /** Send bytes to the specified client */
        void sendToClient(const std::string & message, uint32_t client = 1, bool base_64 = false) const;

        /** Send bytes to all clients */
        void sendToAllClients(const std::string & message, bool base_64 = false) const;

        /** Shared pointer to NetworkStereo camera instance */
        typedef std::shared_ptr<NetworkStereoCamera> Ptr;
    protected:
        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                            cv::Mat & amp_map, cv::Mat & flag_map) override;

    private:
        /** Stores the latest frame */
        std::unique_ptr<NetworkLib::Server> udpServer;

        /** Stores the latest frame */
        Frame latestFrame;

        /** The update callback, null if none set */
        StereoUpdateCallback updCallback;

        /** Mutex to ensure thread safety while updating frame */
        mutable std::mutex frameMutex;

        /** Stores UDP port server is running on */
        int port;
    };
}
