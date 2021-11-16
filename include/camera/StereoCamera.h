#pragma once

#include <opencv2/core.hpp>
#include "Version.h"
#include "camera/DepthCamera.h"

namespace ark {
    /** Stores stereo calibration parameters
     * (not for calibrating stereo cameras; see calib.py for that)
     * run OpenCV calibrateCamera (cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2)
     *                             -> stereoCalibrate -> stereoRectify (R1, R2, P1, P2, Q)
     * 1-left 2-right
     */
    struct StereoCalibration {
        /** size of undistorted images */
        cv::Size undistortedSize;

        /** size of rectified images */
        cv::Size imageSize;

        /** see OpenCV docs for details on different matrices */
        cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, R1, R2, P1, P2, Q;

        /** Default constructor */
        StereoCalibration() { }

        /** load calibration from OpenCV yaml file.
          * should contain fields: imageWidth imageHeight undistortedWidth undistortedHeight cameraMatrix1 cameraMatrix2 distCoeffs1 distCoeffs2 R1 R2 P1 P2 Q
          */
        explicit StereoCalibration(const std::string & yaml_file);

        /** load calibration from OpenCV yaml file.
          * should contain fields: imageWidth imageHeight undistortedWidth undistortedHeight cameraMatrix1 cameraMatrix2 distCoeffs1 distCoeffs2 R1 R2 P1 P2 Q
          */
        void load(const std::string & yaml_file);

        /** save calibration to OpenCV yaml file.
          * should contain fields: cameraMatrix1 cameraMatrix2 distCoeffs1 distCoeffs2 R1 R2 P1 P2 Q
          */
        void save(const std::string & yaml_file) const;

        static std::shared_ptr<StereoCalibration> create() { return std::make_shared<StereoCalibration>(); }

        static std::shared_ptr<StereoCalibration> create(const std::string & yaml_file) { return std::make_shared<StereoCalibration>(yaml_file); }
        typedef std::shared_ptr<StereoCalibration> Ptr;
    };

    /** Configuration for OpenCV SGBM, please see OpenCV documentation */
    struct SGBMConfig {
        // see OpenCV doc for info on what these do
        int disparities = 96;
        int windowSize = 2;
        int preFilterCap = 41;
        int minDisparity = 0;
        int uniquenessRatio = 5;
        int speckleWindowSize = 10;
        int speckleRange = 1;
        int dispL2MaxDiff = 0;

        /* amount to erode/dilate image by after */
        int erodeDilateSize = 1, medianBlurSize = 3;
        double scaleAmount = 18.0;

        static std::shared_ptr<SGBMConfig> create() { return std::make_shared<SGBMConfig>(); }
        typedef std::shared_ptr<SGBMConfig> Ptr;
    };

    /**
    * Class characterizing a generic depth-from-stereo system
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class StereoCamera : public DepthCamera
    {
    public:
        /**
        * Public constructor initializing the stereo camera.
        * @param calib camera calibration information
        * @param imageSource a function that returns the most recent image from the camera when called. Should
        *                  return single grayscale image in CV_8U format (horizontally concatenated left/right images)
        * @param sgbmConf optionally, custom SGBM configuration
        */
        explicit StereoCamera(StereoCalibration::Ptr calib, std::function<cv::Mat(void)> imageSource, SGBMConfig::Ptr sgbmConf = nullptr);

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /**
         * Returns the width of the Stereo camera frame
         */
        int getWidth() const override;

        /**
         * Returns the height of the Stereo camera frame
         */
        int getHeight() const override;

        /**
         * Returns default detection parameters for this depth camera class
         */
        const DetectionParams::Ptr & getDefaultParams() const override;

        /**
         * Returns true if an RGB image is available from this camera.
         * @return true if an RGB image is available from this camera.
         */
        bool hasRGBMap() const override;

        /** Set camera calibration */
        void setCalibration(StereoCalibration::Ptr calib);

        /** Set SGMBM configuration */
        void setSGBMConfig(SGBMConfig::Ptr sgbmConf);

        /** Shared pointer to Stereo camera instance */
        typedef std::shared_ptr<StereoCamera> Ptr;

    protected:
        /**
        * Protected constructor initializing the stereo camera (advanced: create without image source).
        * @param calib camera calibration information
        * @param sgbmConf optionally, custom SGBM configuration
        */
        explicit StereoCamera(StereoCalibration::Ptr calib, SGBMConfig::Ptr sgbmConf = nullptr);


        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        virtual void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
            cv::Mat & amp_map, cv::Mat & flag_map) override;

        /** helper for converting raw to calibrated left, right images */
        void calibrateImage(cv::Mat img_l, cv::Mat img_r, cv::Mat & calibrated_img_l, cv::Mat & calibrated_img_r);

        /** compute XYZ map from frame
          * @param horizontally concatenated left, right images, (rows, cols) = (imageSize.height, imageSize.width*2)
          * @param left_calibrated_out optionally, outputs calibrated left image to this pointer */
        cv::Mat computeDepthSGBM(cv::Mat frame, cv::Mat * left_calibrated_out = NULL);

        /** compute calibrated left image from frame */
        cv::Mat computeImageLeftCalibrated(cv::Mat frame);

        /** helper for splitting horizontally concatenated image to separate left/right images */
        void splitImage(cv::Mat hcat, cv::Mat & img_l, cv::Mat & img_r);

        /** calibration info */
        StereoCalibration::Ptr calib;

        /** SGBM configuration */
        SGBMConfig::Ptr sgbmConf;

        /** OpenCV undistort-rectify maps  */
        cv::Mat rmap[2][2];

    private:
        std::function<cv::Mat(void)> imageSource;
    };
}

