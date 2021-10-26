#pragma once

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
        CameraCalibration(Eigen::Matrix4f transform, rs2_intrinsics intr);

        CameraCalibration();

        void write(cv::FileStorage& fs) const;                        //Write serialization for this class

        void read(const cv::FileNode& node);                          //Read serialization for this class
    };

    //These write and read functions must be defined for the serialization in FileStorage to work
    static void write(cv::FileStorage& fs, const std::string&, const CameraCalibration& x);

    static void read(const cv::FileNode& node, ark::CameraCalibration& x, const ark::CameraCalibration& default_value = ark::CameraCalibration());

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
        Recorder(std::string cam_name);


        ///
        /// Writes camera information to yaml file
        ///
        void write_camera_intrinsics_and_extrinsics(const std::vector<CameraCalibration>& cameras);

        void write_additional_intrinsics_and_extrinsics(const std::vector<CameraCalibration>& cameras);

        void write_camera_params();

        void write_imu_intrinsics();

        void write_additional_parameters();

        /// 
        /// Close files
        ///
        void close();
        
        ~Recorder();
    };


    
};
