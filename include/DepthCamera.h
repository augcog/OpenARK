#pragma once

#include "Version.h"

#include <mutex>
#include <map>

#include "Util.h"
#include "FrameObject.h"
#include "Hand.h"
#include "FramePlane.h"
#include "DetectionParams.h"

namespace ark {
    /**
     * Abstract class defining general behavior of a depth camera.
     * Any depth camera should be able to generate a XYZMap, AmpMap (confidence), and FlagMap.
     */
    class DepthCamera
    {
        // Section A: Methods that should be implemented in child camera classes
    public:

        /**
         * Get the camera's model name.
         */
        virtual const std::string getModelName() const;

        /**
         * Returns the width of the frame in pixels. 
         */
        virtual int getWidth() const = 0;

        /**
         * Returns the height of the frame in pixels. 
         */
        virtual int getHeight() const = 0;
        
        /**
         * Returns default detection parameters for this depth camera class
         */
        virtual const DetectionParams::Ptr & getDefaultParams() const;

        /**
         * Destructor for the DepthCamera class (automatically stops capturing)
         */
        virtual ~DepthCamera();

    protected:
        // Section A.1: Protected methods that must be implemented in child camera classes

        /**
         * Retrieve the next frame from the camera, updating the xyz, rgb, ir, etc. images. 
         * NOTE: Method is abstract and must be implemented in child classes.
         * Directly modify the images passed to this function in the update method to update the camera's images.
         * The images needed will already be initialized to getHeight() * getWidth().
         * WARNING: if has***Map() is false for the camera class, then the ***_map is not guarenteed to be initialized.
         *          so for ex. if you plan to enable the RGB map, please override hasRGBMap() to return true, etc.
         * @param [out] xyz_map XYZ map (projection point cloud). CV_32FC3
         * @param [out] rgb_map RGB image. CV_8UC3
         * @param [out] ir_map IR image. CV_8UC1
         * @param [out] amp_map amplitude map. CV_32FC1
         * @param [out] flag_map flag map. CV_8UC1
         */
        virtual void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                            cv::Mat & amp_map, cv::Mat & flag_map) = 0;

    public:
        // Section B: Stuff that may be overridden but don't need to be 

        /**
         * Returns true if an RGB image is available from this camera. 
         */
        virtual bool hasRGBMap() const;

        /**
         * Returns true if an RGB image is available from this camera.
         */
        virtual bool hasIRMap() const;

        /**
         * Returns true if a flag map is available from this camera.
         */
        virtual bool hasAmpMap() const;

        /**
         * Returns true if a flag map is available from this camera.
         */
        virtual bool hasFlagMap() const;

        /**
         * Value that determines the validity of a point with respect to the camera's ampMap.
         */
        virtual int ampMapInvalidFlagValue() const;

        /**
         * Value that determines the validity of a point with respect to the camera's flagMap.
         */
        virtual float flagMapConfidenceThreshold() const;

        /**
         * Check if the camera input is invalid. 
         * @return true on bad input (e.g. error or disconnection), false otherwise
         */
        virtual bool badInput(); 

        // Section C: Generic methods/variables that may be used by all cameras

        /**
         * Retrieve the next frame from the depth camera.
         * Calls the update() function of the derived camera class and resets stored information for the frame.
         * @param removeNoise if true, performs noise removal on the depth image after retrieving it
         * @return true on success, false on bad input
         */
        bool nextFrame(bool remove_noise = true);

        /**
         * Begin capturing frames continuously from this camera on a parallel thread, 
         * capped at a certain maximum FPS.
         * WARNING: throws an error if capture already started.
         * @param fps_cap maximum FPS of capture (-1 to disable)
         * @param removeNoise if true, performs noise removal on the depth image after retrieving it
         * @see endCapture
         * @see isCapturing
         */
        void beginCapture(int fps_cap = -1, bool remove_noise = true);

        /**
         * Stop capturing from this camera.
         * You may use beginCapture() to start capturing again afterwards.
         * Note: this is performed automatically when this instance is destroyed.
         * @see beginCapture
         * @see isCapturing
         */
        void endCapture();

        /**
         * Returns true if the camera is currently capturing, false otherwise.
         * @see beginCapture
         * @see endCapture
         */
        bool isCapturing();

        /**
         * Add a callback function to be called after each frame update.
         * WARNING: may be called from a different thread than the one where the callback is added.
         * @param func the function. Must take exactly one argument--a reference to the updated DepthCamera instance
         * @see removeUpdateCallBack
         * @return unique ID for this callback function, needed for removeUpdateCallback.
         */
        int addUpdateCallback(std::function<void(DepthCamera &)> func);

        /** Remove the update callback function with the specified unique ID. 
         *  (The ID may be obtained from by addUpdateCallback when the callback is added)
         * @see addUpdateCallBack
         */
        void removeUpdateCallback(int id);
        
        /**
         * Returns the size of the camera's frame (getWidth() * getHeight).
         */
        cv::Size getImageSize() const;

        /**
         * Returns the current XYZ map (ordered point cloud) of the camera. 
         * Contains the XYZ position (in meters) of each pixel on the screen.
         * Type: CV_32FC3
         */
        const cv::Mat getXYZMap() const;

		const std::vector<cv::Mat> getXYZMaps() const;

        /**
         * Get the RGB Image from this camera, if available. Else, throws an error.
         * Type: CV_8UC3
         */
        const cv::Mat getRGBMap() const;

		const std::vector<cv::Mat> getRGBMaps() const;

        /**
         * Get the infrared (IR) Image from this camera, if available. Else, throws an error.
         * Type: CV_8UC1
         */
        const cv::Mat getIRMap() const;

        /**
         * Returns the current AmpMap
         * Type: CV_32FC1
         */
        const cv::Mat getAmpMap() const;

        /**
         * Returns the current FlagMap.
         * Type: CV_8UC1
         */
        const cv::Mat getFlagMap() const;

        /**
         * Reads a sample frame from file.
         * @param source the directory which the frame file is stored
         */
        bool readImage(std::string source);

        /**
         * Writes the current frame into file.
         * @param destination the directory which the frame should be written to
         */
        bool writeImage(std::string destination) const;

        /** Shared pointer to depth camera instance */
        typedef std::shared_ptr<DepthCamera> Ptr;

    protected:
		std::vector<cv::Mat> xyzMaps;
		std::vector<cv::Mat> rgbMaps;
        /**
         * Matrix storing the (x,y,z) data of every point in the observable world.
         * Matrix type CV_32FC3
         */
        cv::Mat xyzMap;

        /**
         * Matrix of confidence values of each corresponding point in the world.
         * Matrix type CV_32FC1
         */
        cv::Mat ampMap;

        /**
         * Matrix representing additional information about the points in the world.
         * Matrix type CV_8UC1
         */
        cv::Mat flagMap;

        /**
         * The RGB image from this camera, if available
         * Matrix type CV_8UC3
         */
        cv::Mat rgbMap;

        /**
         * The infrared image from this camera, if available
         * Matrix type CV_8UC1
         */
        cv::Mat irMap;

        /**
         * Stores pointers to planes visible to the camera in the current frame
         */
        std::vector<FramePlane::Ptr> framePlanes;

        /**
         * Stores pointers to hands visible to the camera in the current frame
         */
        std::vector<Hand::Ptr> hands;

        /**
         * True if input is invalid
         * By default, badInput() returns the value of badInputFlag. 
         * badInput()'s behavior may be overridden.
         */
        bool badInputFlag;

    private:
        // Section D: implementation details

        /**
         * Helper for initializing images used by the generic depth camera.
         * Allocates memory for back buffers if required.
         */
        void initializeImages();

        /**
         * Helper for swapping a single back buffer to the foreground.
         * If the image is not available, creates a dummy mat with null value.
         * @param check_func member function pointer to function that, if true on call, buffers are swapped
         *                   if false, a dummy mat is created
         * @param img pointer to foreground image
         * @param buf pointer to back buffer
         */
        void swapBuffer(bool (DepthCamera::* check_func)() const, cv::Mat & img, cv::Mat & buf);

        /**
         * Helper for swapping all back buffers to the foreground. 
         * If an image is not available, creates a dummy mat with null value.
         */
        void swapBuffers();

        /**
         * Removes noise from an XYZMap based on confidence provided in the AmpMap and FlagMap.
         */
        static void removeNoise(cv::Mat & xyzMap, cv::Mat & ampMap, float confidence_thresh);

        /** stores the callbacks functions to call after each update (ID, function) */
        std::map<int, std::function<void(DepthCamera &)> > updateCallbacks;

        /** 
         * helper function supporting the default capturing behavior 
         * @param fps_cap maximum FPS
         * @param interrupt pointer to the interrupt (when true, thread stops)
         * @param remove_noise if true, automatically removes noise
         */
        void captureThreadingHelper(int fps_cap = 60, volatile bool * interrupt = nullptr,
                                    bool remove_noise = true);

        /** interrupt for immediately terminating the capturing thread */
        bool captureInterrupt = true;

        /**
         * Minimum depth of points (in meters). Points under this depth are presumed to be noise. (0.0 to disable)
         * (Defined in DepthCamera.cpp)
         */
        static const float NOISE_FILTER_LOW;

        /**
         * Maximum depth of points (in meters). Points above this depth are presumed to be noise. (0.0 to disable)
         * (Defined in DepthCamera.cpp)
         */
        static const float NOISE_FILTER_HIGH;

        /** Back buffers for various images */
        cv::Mat xyzMapBuf;
        cv::Mat rgbMapBuf;
        cv::Mat irMapBuf;
        cv::Mat ampMapBuf;
        cv::Mat flagMapBuf;

        /** Mutex to ensure thread safety while updating images 
         *  (mutable = modificable even to const methods)
         */
        mutable std::mutex imageMutex;
    };
}