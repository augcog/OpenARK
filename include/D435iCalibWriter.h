#ifndef _CALIB_WRITER_HPP_
#define _CALIB_WRITER_HPP_

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

namespace ark{

    using namespace cv;
    
    struct CameraCalibration{
    private:
        std::vector<int> dimensions;
        std::vector<float> coeffs;
        std::vector<float> focal;
        std::vector<float> principal;
        Eigen::Matrix4f transform;
    public:
        CameraCalibration(Eigen::Matrix4f transform, rs2_intrinsics intr):
        transform(transform)
        {
            dimensions = {intr.width,intr.height};
            coeffs = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3]};
            focal = {intr.fx, intr.fy};
            principal = { intr.ppx, intr.ppy};
        }

        CameraCalibration(){}

        void write(cv::FileStorage& fs) const                        //Write serialization for this class
        {
            fs << "{";
            std::vector<float> t_sc(transform.data(), transform.data() + transform.rows() * transform.cols());
            fs << "T_SC" << t_sc;
            fs << "image_dimension" << dimensions;
            fs << "distortion_type" << "radialtangential";
            fs << "distortion_coefficients" << coeffs;
            fs << "focal_length" << focal;
            fs << "principal_point" << principal;
            fs << "}";
        }

        void read(const cv::FileNode& node)                          //Read serialization for this class
        {
        }


    };

    //These write and read functions must be defined for the serialization in FileStorage to work
    static void write(cv::FileStorage& fs, const std::string&, const CameraCalibration& x)
    {
        x.write(fs);
    }

    static void read(const cv::FileNode& node, ark::CameraCalibration& x, const ark::CameraCalibration& default_value = ark::CameraCalibration()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
    }



    ///
    /// This is a modification of the above bridge code
    /// to send data to a file instead of the slam system
    ///
    class Recorder{
    private:
        Recorder(){}
        //output directory to store data
        cv::FileStorage intr_file_;

    public:

        ///
        /// Create the recorder and attempt to open the file
        /// will happily overwrite existing files. 
        ///
        Recorder(std::string cam_name)
        {
            intr_file_ = cv::FileStorage(cam_name+std::string("_intr.yaml"), cv::FileStorage::WRITE);
        }


        ///
        /// Writes camera information to yaml file
        ///
        void write_camera_intrinsics_and_extrinsics(const std::vector<CameraCalibration>& cameras){
            
            intr_file_ << "cameras" << cameras;
        }

        void write_additional_intrinsics_and_extrinsics(const std::vector<CameraCalibration>& cameras){
            intr_file_ << "additional_cameras" << cameras;
        }

        void write_camera_params(){
            intr_file_ << "camera_params";
            intr_file_ << "{" << "camera_rate" << 30
                << "sigma_absolute_translation" << 0.0f
                << "sigma_absolute_orientation" << 0.0f
                << "sigma_c_relative_translation" << 0.0f
                << "sigma_c_relative_orientation" << 0.0f
                << "timestamp_tolerance" << 0.005
                << "}";
        }

        void write_imu_intrinsics(){
            intr_file_ << "imu_params";
            std::vector<float> acc_bias = {0,0,0};
            std::vector<float> t_bs = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
            intr_file_ << "{" << "a_max" << 176.0
                << "g_max" << 7.8
                << "sigma_g_c" << 7.0710678119212389e-04 //sqrt(intr.gyro.noise_variances[0])
                << "sigma_a_c" << 9.9999997764825821e-03 //sqrt(intr.acc.noise_variances[0])
                << "sigma_bg" << 0.015 //sqrt(intr.gyro.bias_variances[0])
                << "sigma_ba" << 0.38 //sqrt(intr.acc.bias_variances[0])
                << "sigma_gw_c" << 4.0e-6
                << "sigma_aw_c" << 4.0e-5
                << "tau" << 3600.0
                << "g" << 9.81007
                << "a0" << acc_bias
                << "imu_rate" << 200
                << "T_BS" << t_bs
                << "}";
        }

        void write_additional_parameters(){

        std::vector<float> t_wc_w = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        intr_file_
        // Estimator parameters
        << "numKeyframes" << 4 // number of keyframes in optimisation window
        << "numImuFrames" << 3 // number of frames linked by most recent nonlinear IMU error terms

        // ceres optimization options
        << "ceres_options" << "{"
           <<  "minIterations" << 3   // minimum number of iterations always performed
           <<  "maxIterations" << 5  // never do more than these, even if not converged
           <<  "timeLimit" << 0.035   // [s] negative values will set the an unlimited time limit
        << "}"

        // detection
        << "detection_options" << "{"
           <<  "threshold" << 40.0      // detection threshold. By default the uniformity radius in pixels
           <<  "octaves" << 2           // number of octaves for detection. 0 means single-scale at highest resolution
           <<  "maxNoKeypoints" << 75  // restrict to a maximum of this many keypoints per image (strongest ones)
        << "}"

        // delay of images [s]" <<
        << "imageDelay" << 0.000  // in case you are using a custom setup, you will have to calibrate this. 0 for the VISensor.

        // display debug images?
        << "displayImages" << "false"  // displays debug video and keyframe matches. May be slow.

        // use direct driver
        << "useDriver" << "false" 

        << "enableLoopClosureDetection" << true
        << "publishing_options" << "{"
            << "publish_rate" << 200                  // rate at which odometry updates are published only works properly if imu_rate/publish_rate is an integer!!
            << "publishLandmarks" << "false"            // select, if you want to publish landmarks at all
            << "landmarkQualityThreshold" << 1.0e-2   // landmark with lower quality will not be published
            << "maximumLandmarkQuality" << 0.05       // landmark with higher quality will be published with the maximum colour intensity
            << "maxPathLength" << 20                  // maximum length of the published path
            << "publishImuPropagatedState" << "false"    // Should the state that is propagated with IMU messages be published? Or just the optimized ones?
            // provide custom World frame Wc
            << "T_Wc_W" << t_wc_w
            << "trackedBodyFrame" << "B"                // B or S, the frame of reference that will be expressed relative to the selected worldFrame
            << "velocitiesFrame" << "Wc"                // Wc, B or S,  the frames in which the velocities of the selected trackedBodyFrame will be expressed in
        << "}";


        }

        /// 
        /// Close files
        ///
        void close(){

            intr_file_.release();
        }
        
        ~Recorder(){
        }
    };


    
};

#endif