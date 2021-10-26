//
// Created by lucas on 1/28/18.
//
#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "hand_and_avatar/hand/Hand.h"
#include "hand_and_avatar/FramePlane.h"

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
        bool operator<(const ImuMeasurement& right) const
        {
          return timestamp>right.timestamp;
        }
    };

    /** A frame containing an RGB image, a depth image, and a transformation matrix */
    class RGBDFrame {
    public:
        cv::Mat mTcw;
        cv::Mat imRGB;
        cv::Mat imDepth;
        int frameId;
        RGBDFrame(){
            mTcw = cv::Mat::eye(4,4,CV_32FC1);
            frameId = -1;
        }
        RGBDFrame(const RGBDFrame& frame)
        {
            frame.mTcw.copyTo(mTcw);
            frame.imRGB.copyTo(imRGB);
            frame.imDepth.copyTo(imDepth);
            frameId = frame.frameId;
        }

        typedef std::shared_ptr<RGBDFrame> Ptr;
    };

    /** A container for Camera Calibration */
    class CameraCalibration {  
    public:
        cv::Mat camera_matrix;
        cv::Mat distortion_coeffs;

        CameraCalibration(double fx,double fy,double cx,double cy, cv::Mat distortion_coeffs = cv::Mat()):
        distortion_coeffs(distortion_coeffs){
            camera_matrix = cv::Mat(3,3,cv::DataType<double>::type);
            camera_matrix.at<double>(0,0) = fx;
            camera_matrix.at<double>(1,1) = fy;
            camera_matrix.at<double>(0,2) = cx;
            camera_matrix.at<double>(1,2) = cy;
        }

        void setDistortionCoefficients(double k1,double k2, double p1, double p2){
            distortion_coeffs= cv::Mat(4,1,cv::DataType<double>::type);
            distortion_coeffs.at<double>(0) = k1;
            distortion_coeffs.at<double>(1) = k2;
            distortion_coeffs.at<double>(2) = p1;
            distortion_coeffs.at<double>(3) = p2;
        }

        void setDistortionCoefficients(double k1,double k2, double p1, double p2, 
                double k3){
            distortion_coeffs= cv::Mat(5,1,cv::DataType<double>::type);
            distortion_coeffs.at<double>(0) = k1;
            distortion_coeffs.at<double>(1) = k2;
            distortion_coeffs.at<double>(2) = p1;
            distortion_coeffs.at<double>(3) = p2;
            distortion_coeffs.at<double>(4) = k3;
        }

        void setDistortionCoefficients(double k1,double k2, double p1, double p2, 
                double k3, double k4, double k5, double k6){
            distortion_coeffs= cv::Mat(8,1,cv::DataType<double>::type);
            distortion_coeffs.at<double>(0) = k1;
            distortion_coeffs.at<double>(1) = k2;
            distortion_coeffs.at<double>(2) = p1;
            distortion_coeffs.at<double>(3) = p2;
            distortion_coeffs.at<double>(4) = k3;
            distortion_coeffs.at<double>(5) = k4;
            distortion_coeffs.at<double>(6) = k5;
            distortion_coeffs.at<double>(7) = k6;
        }

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

        MapKeyFrame():
        frameId_(-1),optimized_(false),previousKeyframeId_(-1){

        }

        const std::vector<cv::KeyPoint>& keypoints(int cameraIdx){
            return keypoints_[cameraIdx];
        }

        const cv::Mat& descriptors(int cameraIdx){
            return descriptors_[cameraIdx];
        }

        const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> & homogeneousKeypoints3d(int cameraIdx){ 
            return keypoints3dh_C[cameraIdx];
        }

        void descriptorsAsVec(int cameraIdx, std::vector<cv::Mat>& out){
            out.clear();
            out.reserve(descriptors_[cameraIdx].rows);
            for(int i=0; i<descriptors_[cameraIdx].rows; i++){
              out.emplace_back(descriptors_[cameraIdx].row(i));
            } 
        }

        void setOptimizedTransform(const Eigen::Matrix4d& T_WS_in){

            T_WS_Optimized_ = T_WS_in;
            optimized_ = true;
        }

        Eigen::Matrix4d T_WS(){
            if(optimized_){
                return T_WS_Optimized_;
            }else
                return T_WS_;
        }

        Eigen::Matrix4d T_WC(int index)
        {
            if(index>=0 && index<T_SC_.size()){
                return T_WS()*T_SC_[index];
            }else
                return T_WS();
        }

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

        bool getImageByType(FrameType type, cv::Mat& out, int num=0 ){
            int found =0;
            for(size_t i=0; i<images_.size(); i++){
                if(image_types_[i] == type){
                    if(found==num){
                        out = images_[i];
                        return true;
                    }else{
                        found++;
                    }
                }
            } 
            return false;
        }

        bool getImage(cv::Mat& out, int num){
            if(num<images_.size()){
                out=images_[num];
                return true;
            }
            return false;
        }

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
        explicit MultiCameraFrame(int frame_id = -1)
        {
            frameId_ = frame_id;
        }

        /** Construct a MultiCameraFrame from the given images,
          * camera system, and frame ID*/
        MultiCameraFrame(std::vector<cv::Mat> images, const Eigen::Matrix4d& T_KS, 
            int frame_id = -1, int keyframe_id = -1){
            this->images_ = images;
            this->T_KS_ = T_KS;
            frameId_ = frame_id;
            keyframeId_ = keyframe_id;
        } 

        /** Construct a MultiCameraFrame from the given image,
            and camera system, and frame ID */
        MultiCameraFrame(cv::Mat image, const Eigen::Matrix4d& T_KS,
            int frame_id = -1, int keyframe_id = -1){
            images_.push_back(image);
            this->T_KS_ = T_KS;
            frameId_ = frame_id;
            keyframeId_ = keyframe_id;
        }

        //make threadsafe setting of transform

        //make threadsafe
        Eigen::Matrix4d T_WS()
        {
            if(keyframeId_!=-1 && keyframe_.get()!=nullptr)
                return keyframe_->T_WS()*T_KS_;
            else 
                return T_KS_;
        }

        Eigen::Matrix4d T_WC(int index)
        {
            if(index>=0 && index<T_SC_.size()){
                return T_WS()*T_SC_[index];
            }else
                return T_WS();
        }

        void saveSimple(std::string dir){
            std::stringstream filename;
            for(size_t i=0; i < images_.size(); i++){
                filename << dir << frameId_ << "_" << i << ".png";
                cv::imwrite(filename.str(),images_[i]);
            }

            //Add timesync data to timesync file
            std::ofstream kf_T;
            kf_T.open(dir+std::to_string(frameId_)+"_T.csv");
            Eigen::Matrix4d T = T_WS();
            kf_T << T(0,0) << "," << T(0,1) << "," <<  T(0,2) << "," << T(0,3)
            << "," << T(1,0) << "," << T(1,1) << "," <<  T(1,2) << "," << T(1,3)
            << "," << T(2,0) << "," << T(2,1) << "," <<  T(2,2) << "," << T(2,3)
            << "," << T(3,0) << "," << T(3,1) << "," <<  T(3,2) << "," << T(3,3);
            kf_T.close();

        }

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
