//
// Created by lucas on 1/28/18.
//
#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "Hand.h"
#include "FramePlane.h"

namespace ark{

    typedef pcl::PointXYZRGB PointType;

    /** Container for storing syncronized imu measurements */
    struct ImuPair{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp;
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
    };

    /** Container for storing measurement from a single sensor */
    enum class ImuMeasurementType {gyro,accel};
    struct ImuMeasurement{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp;
        Eigen::Vector3d data;
        ImuMeasurementType type;
        bool operator<(const ImuMeasurement& right) const;
    };

    /** A frame containing an RGB image, a depth image, and a transformation matrix */
    class RGBDFrame {
    public:
        cv::Mat mTcw;
        cv::Mat imRGB;
        cv::Mat imDepth;
        int frameId;
        RGBDFrame();
        RGBDFrame(const RGBDFrame& frame);

        typedef std::shared_ptr<RGBDFrame> Ptr;
    };

    /** A container for Camera Calibration */
    class CameraCalibration {  
    public:
        cv::Mat camera_matrix;
        cv::Mat distortion_coeffs;

        CameraCalibration(double fx,double fy,double cx,double cy, cv::Mat distortion_coeffs = cv::Mat());

        void setDistortionCoefficients(double k1,double k2, double p1, double p2);

        void setDistortionCoefficients(double k1,double k2, double p1, double p2, 
                double k3);

        void setDistortionCoefficients(double k1,double k2, double p1, double p2, 
                double k3, double k4, double k5, double k6);

    };//CameraCalibration

    /** A paired down MultiCameraFrame, only containing information necessary to be stored by the map */
    class MapKeyFrame{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<MapKeyFrame> Ptr;
        /** ID of the frame (may be -1 if not available) */
        int frameId_;
        /** Timestamp */
        double timestamp_;
        /** Original world position of the Keyframe */
        Eigen::Matrix4d T_WS_; 
        /** Optimized world position of the Keyframe */
        Eigen::Matrix4d T_WS_Optimized_; 
        /** Bool checking whether the frame has been optimized */
        bool optimized_;
        /** Keypoints and Descriptors extracted by the SLAM System 
         ** A vector of keypoints is kept for each image
         ** A cv::Mat of all descriptors is kept for each image */
        std::vector<std::vector<cv::KeyPoint > > keypoints_; 
        std::vector<cv::Mat> descriptors_;
        /** Estimated 3D feature positions of all keypoints in each image */
        std::vector<std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>> keypoints3dh_C; 
        /** ID of the previous keyframe (may be -1 if not available) */
        int previousKeyframeId_;
        /** Pointer to the keyframe (may be nullptr if not available) */
        MapKeyFrame::Ptr previousKeyframe_;

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> T_SC_; 

        MapKeyFrame();

        const std::vector<cv::KeyPoint>& keypoints(int cameraIdx);

        const cv::Mat& descriptors(int cameraIdx);

        const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> & homogeneousKeypoints3d(int cameraIdx);

        void descriptorsAsVec(int cameraIdx, std::vector<cv::Mat>& out);

        void setOptimizedTransform(const Eigen::Matrix4d& T_WS_in);

        Eigen::Matrix4d T_WS();

        Eigen::Matrix4d T_WC(int index);

    };

    enum class FrameType { Depth, IR, RGB, XYZMap };

    /** A set of images taken on the same frame, possibly by multiple instruments/cameras */
    class MultiCameraFrame { 
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<MultiCameraFrame> Ptr;

        /** Timestamp of the frame (may be -1 if not available) */
        double timestamp_;

        /** Vector of images in the frame */
        std::vector<cv::Mat> images_;
        /** Image type */
        std::vector<FrameType> image_types_;
        /** Mat format */
        std::vector<int> image_mat_format_;

        bool getImageByType(FrameType type, cv::Mat& out, int num=0 );

        bool getImage(cv::Mat& out, int num);

        /** Transformation matrix of the sensor body in keyframe coordinates 
         ** This may be Identity if transforms not available
         ** This should be set to world coordinates if keyframeId is -1 */
        Eigen::Matrix4d T_KS_; 
        /** Tranformation matrices of the cameras WRT sensor body */
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> T_SC_; 

        /** ID of the frame (may be -1 if not available) */
        int frameId_;

        /** ID of the keyframe for this frame (may be -1 if not available) */
        int keyframeId_;

        /** Pointer to keyframe for this frame (may be nullptr if not available) */
        MapKeyFrame::Ptr keyframe_;

        std::vector<Hand::Ptr> hands_;
        std::vector<FramePlane::Ptr> planes_;


        /** Construct an empty MultiCameraFrame with the given ID (default -1) */
        explicit MultiCameraFrame(int frame_id = -1);

        /** Construct a MultiCameraFrame from the given images,
          * camera system, and frame ID*/
        MultiCameraFrame(std::vector<cv::Mat> images, const Eigen::Matrix4d& T_KS, 
            int frame_id = -1, int keyframe_id = -1); 

        /** Construct a MultiCameraFrame from the given image,
            and camera system, and frame ID */
        MultiCameraFrame(cv::Mat image, const Eigen::Matrix4d& T_KS,
            int frame_id = -1, int keyframe_id = -1);

        //make threadsafe setting of transform

        //make threadsafe
        Eigen::Matrix4d T_WS();

        Eigen::Matrix4d T_WC(int index);

        void saveSimple(std::string dir);

        //get image by type

        /*Eigen::Matrix4d T_WC(int cameraIdx){
            if(cameraSystem_.get()!=nullptr){
                return T_WS()*cameraSystem_->getExtrinsicFromBody(cameraIdx);
            }else
                return T_WS();
        }*/

    };

    struct CameraParameter {
        int width = 640;
        int height = 480;
        float emitterPower = 0.5f;
        int irDepthFps = 30;
        int imuFps = 200;
    };
}
