#pragma once

#include "version.h"

#include "Util.h"
#include "FrameObject.h"
#include "Hand.h"
#include "FramePlane.h"
#include "ObjectParams.h"

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
         * @param fps_cap maximum FPS of capture.
         * @param removeNoise if true, performs noise removal on the depth image after retrieving it
         * @see endCapture
         * @see isCapturing
         */
        void beginCapture(int fps_cap = 60, bool remove_noise = true);

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
         * Returns the size of the camera's frame (getWidth() * getHeight).
         */
        cv::Size getImageSize() const;

        /*
         * Retrieve a list of hands visible in the current frame
         * @see getFramePlanes
         * @see getFrameObjects
         * @param params parameters for object/plane/hand detection
         * @param elim_planes if true, finds and eliminates planes in the scene
         * @return vector of shared pointers to visible hands
         */
        std::vector<HandPtr> & getFrameHands(const ObjectParams * params = nullptr, bool elim_planes = true);
 
        /*
         * Retrieve a list of planes visible in the current frame
         * @see getFrameHands
         * @see getFrameObjects
         * @param params parameters for object/plane/hand detection
         * @return vector of shared pointers to visible planes
         */
        std::vector<FramePlanePtr> & getFramePlanes(const ObjectParams * params = nullptr);

        /*
         * Retrieve a list of objects visible in the current frame
         * @see getFrameHands
         * @see getFramePlanes
         * @param params parameters for object/plane/hand detection
         * @return vector of shared pointers to visible objects
         */
        std::vector<FrameObjectPtr> & getFrameObjects(const ObjectParams * params = nullptr);

        /**
         * Returns the current XYZ map (ordered point cloud) of the camera. 
         * Contains the XYZ position (in meters) of each pixel on the screen.
         * Type: CV_32FC3
         */
        const cv::Mat getXYZMap() const;

        /**
         * Returns the current normal map (surface normal vectors at each point).
         * If not already cached, this is computed automatically from the depth map when requested.
         * Type: CV_32FC3
         */
        const cv::Mat getNormalMap();

        /**
         * Get the RGB Image from this camera, if available. Else, throws an error.
         * Type: CV_8UC3
         */
        const cv::Mat getRGBMap() const;

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

    protected:
        /**
         * Matrix storing the (x,y,z) data of every point in the observable world.
         * Matrix type CV_32FC3
         */
        cv::Mat xyzMap;

        /**
         * Matrix storing the surface normal vectors (facing viewer) at each point in the observable world.
         * This is computed automatically from the depth map if required.
         * Implementers of subclasses do not need to deal with this at all.
         * Matrix type CV_32FC3
         */
        cv::Mat normalMap;

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
         * Stores pointers to all objects visible to the camera in the current frame
         */
        std::vector<FrameObjectPtr> frameObjects;

        /**
         * Stores pointers to planes visible to the camera in the current frame
         */
        std::vector<FramePlanePtr> framePlanes;

        /**
         * Stores pointers to hands visible to the camera in the current frame
         */
        std::vector<HandPtr> hands;

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
         * helper function for computing the normal map of the current depth image
         * @param params parameters
         */
        void computeNormalMap(const ObjectParams * params = nullptr);

        /**
         * Removes noise from an XYZMap based on confidence provided in the AmpMap and FlagMap.
         */
        static void removeNoise(cv::Mat & xyzMap, cv::Mat & ampMap, float confidence_thresh);

        /** 
         * helper function supporting the default capturing behavior 
         * @param fps_cap maximum FPS
         * @param interrupt pointer to the interrupt (when true, thread stops)
         * @param remove_noise if true, automatically removes noise
         */
        void captureThreadingHelper(int fps_cap = 60, volatile bool * interrupt = nullptr,
                                    bool remove_noise = true);

        /**
         * helper function for the equations of all planes in a frame given its xyz and normal map s.
         * @param[in] xyz_map the frame's xyz map
         * @param[out] output_equations vector to be filled with equations of planes (in the form ax + by - z + c = 0)
         * @param[out] output_points vector to be filled with vectors of ij coordinate points on planes
         * @param[out] output_points_xyz vector to be filled with vectors of xyz coordinate points on planes
         * @param[in] normal_map the frame's normal map (by default, uses the camera's getNormalMap)
         * @param[in] fill_mask optional mask for limiting which points may be used in plane detection
         * @param[in] fill_color color on fill_mask to allow detection
         * @param[in] params plane detection parameters
         */
        void detectPlaneHelper(const cv::Mat & xyz_map,  std::vector<Vec3f> & output_equations,
            std::vector<VecP2iPtr> & output_points, std::vector<VecV3fPtr> & output_points_xyz,
            const ObjectParams * params = nullptr, const cv::Mat * normal_map = nullptr, 
            const cv::Mat * fill_mask = nullptr, uchar fill_color = 0);

        /**
         * helper function for the equations of all planes in a frame given its xyz and normal map s.
         * @param[in] xyz_map the frame's xyz map
         * @param[out] output_hands vector to push detected hands to
         * @param[in] params hand detection parameters
         * @param[in, out] best_hand_dist optional float value for storing the distance to the
         *                                closest hand
         * @param[out] pending_mask optional mask to record "pending" clusters 
         *             (clusters that have not been eliminated but are not hands),
         *             points in the first cluster are given value 255, second 254, etc.
         * @param[out] pending_count optional output for the number of pending clusters
         * @param[in] fill_mask optional mask for limiting which points may be used in hand detection
         * @param[in] fill_color color on fill_mask to allow detection
         * @return shared pointer to current best Hand instance.
         */
        boost::shared_ptr<Hand> detectHandHelper(const cv::Mat & xyz_map,
            std::vector<HandPtr> & output_hands,
            const ObjectParams * params = nullptr, float * best_hand_dist = nullptr,
            cv::Mat * pending_mask = nullptr, uchar * pending_count = nullptr,
            const cv::Mat * fill_mask = nullptr, uchar fill_color = 0);

        /** interrupt for immediately terminating the capturing thread */
        bool captureInterrupt = true;

        /** Flag for recording if objects, hands, etc.
          * have already been computed in this frame */
        uint isCached = 0;

        /** Binary bit values for use with the 'isCached' flag */
        static const uint FLAG_FRM_OBJS = 1, FLAG_FRM_HANDS = 2, FLAG_FRM_PLANES = 4, FLAG_NORMALS = 256;

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