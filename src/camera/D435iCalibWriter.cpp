
#include <librealsense2/rs.hpp> 
#include <Eigen/Core>   
#include <iostream>
#include "openark/camera/D435iCalibWriter.h"

namespace ark {
    CameraCalibration::CameraCalibration(Eigen::Matrix4f transform, rs2_intrinsics intr) :
        transform(transform)
    {
        dimensions = { intr.width,intr.height };
        coeffs = { intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3] };
        focal = { intr.fx, intr.fy };
        principal = { intr.ppx, intr.ppy };
    }

    CameraCalibration::CameraCalibration() {}

    void CameraCalibration::write(cv::FileStorage& fs) const          //Write serialization for this class
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

    void CameraCalibration::read(const cv::FileNode& node)                          //Read serialization for this class
    {
    }

    //These write and read functions must be defined for the serialization in FileStorage to work
    static void write(cv::FileStorage& fs, const std::string&, const CameraCalibration& x)
    {
        x.write(fs);
    }

    static void read(const cv::FileNode& node, ark::CameraCalibration& x, const ark::CameraCalibration& default_value = ark::CameraCalibration()) {
        if (node.empty())
            x = default_value;
        else
            x.read(node);
    }

    ///
    /// Create the recorder and attempt to open the file
    /// will happily overwrite existing files. 
    ///
    Recorder::Recorder(std::string cam_name)
    {
        intr_file_ = cv::FileStorage(cam_name + std::string("_intr.yaml"), cv::FileStorage::WRITE);
    }

    ///
    /// Writes camera information to yaml file
    ///
    void Recorder::write_camera_intrinsics_and_extrinsics(const std::vector<CameraCalibration>& cameras) {

        intr_file_ << "cameras" << cameras;
    }

    void Recorder::write_additional_intrinsics_and_extrinsics(const std::vector<CameraCalibration>& cameras) {
        intr_file_ << "additional_cameras" << cameras;
    }

    void Recorder::write_camera_params() {
        intr_file_ << "camera_params";
        intr_file_ << "{" << "camera_rate" << 30
            << "sigma_absolute_translation" << 0.0f
            << "sigma_absolute_orientation" << 0.0f
            << "sigma_c_relative_translation" << 0.0f
            << "sigma_c_relative_orientation" << 0.0f
            << "timestamp_tolerance" << 0.005
            << "}";
    }

    void Recorder::write_imu_intrinsics() {
        intr_file_ << "imu_params";
        std::vector<float> acc_bias = { 0,0,0 };
        std::vector<float> t_bs = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
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

    void Recorder::write_additional_parameters() {

        std::vector<float> t_wc_w = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
        intr_file_
            // Estimator parameters
            << "numKeyframes" << 4 // number of keyframes in optimisation window
            << "numImuFrames" << 3 // number of frames linked by most recent nonlinear IMU error terms

                                   // ceres optimization options
            << "ceres_options" << "{"
            << "minIterations" << 3   // minimum number of iterations always performed
            << "maxIterations" << 5  // never do more than these, even if not converged
            << "timeLimit" << 0.035   // [s] negative values will set the an unlimited time limit
            << "}"

            // detection
            << "detection_options" << "{"
            << "threshold" << 40.0      // detection threshold. By default the uniformity radius in pixels
            << "octaves" << 2           // number of octaves for detection. 0 means single-scale at highest resolution
            << "maxNoKeypoints" << 75  // restrict to a maximum of this many keypoints per image (strongest ones)
            << "}"

            // delay of images [s]" <<
            << "imageDelay" << 0.000  // in case you are using a custom setup, you will have to calibrate this. 0 for the VISensor.

                                      // display debug images?
            << "displayImages" << "false"  // displays debug video and keyframe matches. May be slow.

                                           // use direct driver
            << "useDriver" << "false"

            << "enableLoopClosureDetection" << true
            << "Recon_VoxelSize" << 0.01
            << "Recon_BlockSize" << 2.0
            << "Recon_MaxDepth" << 2.0
            << "Recon_SaveFrames" << true
            << "Recon_MeshWinWidth" << 1000
            << "Recon_MeshWinHeight" << 1000

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
    void Recorder::close() {

        intr_file_.release();
    }

    Recorder::~Recorder() {
    }
}

int main(int argc, char *argv[]) try
{
    //ensure an output directory is supplied
    if (argc != 2){ 
        std::cout<<"usage: "<< argv[0] <<" <camera name>\n"<< 
        "Intrisics file will be saved as <camera name>_intr.yaml\n";
        return 0;
    }

    std::string cam_name(argv[1]);

    ark::Recorder rec(cam_name);


    //Set up realsense pipeline
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,-1,640,480,RS2_FORMAT_Z16,30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    //rs2::config cfg2;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,250);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,200);

    cfg.enable_stream(RS2_STREAM_COLOR,-1,640,480,RS2_FORMAT_BGR8,30);

    rs2::pipeline pipe;
    //rs2::pipeline motion_pipe;

    // Start streaming with default recommended configuration
    rs2::pipeline_profile selection = pipe.start(cfg);

    // Get image streams
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>();

    auto ir1_stream = selection.get_stream(RS2_STREAM_INFRARED,1)
                             .as<rs2::video_stream_profile>();

    auto ir2_stream = selection.get_stream(RS2_STREAM_INFRARED,2)
                             .as<rs2::video_stream_profile>();
    auto color_stream = selection.get_stream(RS2_STREAM_COLOR)
                             .as<rs2::video_stream_profile>();
    // Get motion streams
    auto gyro_stream =  selection.get_stream(RS2_STREAM_GYRO)
                             .as<rs2::motion_stream_profile>();
    auto accel_stream =  selection.get_stream(RS2_STREAM_ACCEL)
                             .as<rs2::motion_stream_profile>();

    //get extrinsics between cameras and imu
    rs2_extrinsics extrinsics = ir1_stream.get_extrinsics_to(accel_stream);
    Eigen::Vector3f T_12(extrinsics.translation[0], extrinsics.translation[1], extrinsics.translation[2]);
    Eigen::Map<Eigen::Matrix3f>R_12(extrinsics.rotation);

    Eigen::Matrix4f tf_ir1 = Eigen::Matrix4f::Identity();
    tf_ir1.block<3,3>(0,0)=R_12;
    tf_ir1.block<3,1>(0,3)=T_12;

    rs2_extrinsics extrinsics2 = ir2_stream.get_extrinsics_to(accel_stream);
    Eigen::Vector3f T2_12(extrinsics2.translation[0], extrinsics2.translation[1], extrinsics2.translation[2]);
    Eigen::Map<Eigen::Matrix3f>R2_12(extrinsics2.rotation);
    Eigen::Matrix4f tf_ir2 = Eigen::Matrix4f::Identity();
    tf_ir2.block<3,3>(0,0)=R2_12;
    tf_ir2.block<3,1>(0,3)=T2_12;

    rs2_extrinsics extrinsics3 = depth_stream.get_extrinsics_to(accel_stream);
    Eigen::Vector3f T3_12(extrinsics3.translation[0], extrinsics3.translation[1], extrinsics3.translation[2]);
    Eigen::Map<Eigen::Matrix3f>R3_12(extrinsics3.rotation);
    Eigen::Matrix4f tf_depth = Eigen::Matrix4f::Identity();
    tf_depth.block<3,3>(0,0)=R3_12;
    tf_depth.block<3,1>(0,3)=T3_12;

    rs2_extrinsics extrinsics4 = color_stream.get_extrinsics_to(accel_stream);
    Eigen::Vector3f T4_12(extrinsics4.translation[0], extrinsics4.translation[1], extrinsics4.translation[2]);
    Eigen::Map<Eigen::Matrix3f>R4_12(extrinsics4.rotation);
    Eigen::Matrix4f tf_color = Eigen::Matrix4f::Identity();
    tf_color.block<3,3>(0,0)=R4_12;
    tf_color.block<3,1>(0,3)=T4_12;

    //get camera intrinsics
    rs2_intrinsics rs_intr_ir1 = ir1_stream.get_intrinsics();
    rs2_intrinsics rs_intr_ir2 = ir2_stream.get_intrinsics();
    rs2_intrinsics rs_intr_depth = depth_stream.get_intrinsics();
    rs2_intrinsics rs_intr_color = color_stream.get_intrinsics();


    //compose calibration for slam cameras
    ark::CameraCalibration ir1_calib(tf_ir1.transpose(),rs_intr_ir1);
    ark::CameraCalibration ir2_calib(tf_ir2.transpose(),rs_intr_ir2);

    std::vector<ark::CameraCalibration> camera_calibs;
    camera_calibs.push_back(ir1_calib);
    camera_calibs.push_back(ir2_calib);

    //Write intrinsics to file
    rec.write_camera_intrinsics_and_extrinsics(camera_calibs);

    //compose calibration for additional cameras
    ark::CameraCalibration depth_calib(tf_depth.transpose(),rs_intr_depth);
    ark::CameraCalibration color_calib(tf_color.transpose(),rs_intr_color);

    std::vector<ark::CameraCalibration> add_camera_calibs;
    add_camera_calibs.push_back(depth_calib); 
    add_camera_calibs.push_back(color_calib); 
    rec.write_additional_intrinsics_and_extrinsics(add_camera_calibs);

    rec.write_camera_params();
    rec.write_imu_intrinsics();
    rec.write_additional_parameters();

    Eigen::Matrix3f cam_mat_ir1;
    cam_mat_ir1(0,0) = rs_intr_ir1.fx;
    cam_mat_ir1(0,2) = rs_intr_ir1.ppx;
    cam_mat_ir1(1,1) = rs_intr_ir1.fy;
    cam_mat_ir1(1,2) = rs_intr_ir1.ppy;
    cam_mat_ir1(2,2) = 1;

    Eigen::Matrix3f cam_mat_ir2;
    cam_mat_ir2(0,0) = rs_intr_ir2.fx;
    cam_mat_ir2(0,2) = rs_intr_ir2.ppx;
    cam_mat_ir2(1,1) = rs_intr_ir2.fy;
    cam_mat_ir2(1,2) = rs_intr_ir2.ppy;
    cam_mat_ir2(2,2) = 1;

    std::cout << "IR1: " << cam_mat_ir1.inverse() << std::endl;
    std::cout << "IR2: " << cam_mat_ir2.inverse() <<  std::endl;


    std::cout << "----------------Camera 1----------------\n";
    std::cout << "Principal Point         : " << rs_intr_ir1.ppx << ", " << rs_intr_ir1.ppy << std::endl;
    std::cout << "Focal Length            : " << rs_intr_ir1.fx << ", " << rs_intr_ir1.fy << std::endl;
    std::cout << "Distortion Model        : " << rs_intr_ir1.model << std::endl;
    std::cout << "Distortion Coefficients : [" << rs_intr_ir1.coeffs[0] << "," << rs_intr_ir1.coeffs[1] << "," <<
        rs_intr_ir1.coeffs[2] << "," << rs_intr_ir1.coeffs[3] << "," << rs_intr_ir1.coeffs[4] << "]" << std::endl;

    std::cout << "----------------Camera 2----------------\n";
    std::cout << "Principal Point         : " << rs_intr_ir2.ppx << ", " << rs_intr_ir2.ppy << std::endl;
    std::cout << "Focal Length            : " << rs_intr_ir2.fx << ", " << rs_intr_ir2.fy << std::endl;
    std::cout << "Distortion Model        : " << rs_intr_ir2.model << std::endl;
    std::cout << "Distortion Coefficients : [" << rs_intr_ir2.coeffs[0] << "," << rs_intr_ir2.coeffs[1] << "," <<
        rs_intr_ir2.coeffs[2] << "," << rs_intr_ir2.coeffs[3] << "," << rs_intr_ir2.coeffs[4] << "]" << std::endl;
    std::cout << "----------------Depth----------------\n";
    std::cout << "Principal Point         : " << rs_intr_depth.ppx << ", " << rs_intr_depth.ppy << std::endl;
    std::cout << "Focal Length            : " << rs_intr_depth.fx << ", " << rs_intr_depth.fy << std::endl;
    std::cout << "Distortion Model        : " << rs_intr_depth.model << std::endl;
    std::cout << "Distortion Coefficients : [" << rs_intr_depth.coeffs[0] << "," << rs_intr_depth.coeffs[1] << "," <<
        rs_intr_depth.coeffs[2] << "," << rs_intr_depth.coeffs[3] << "," << rs_intr_depth.coeffs[4] << "]" << std::endl;
    std::cout << "----------------Color----------------\n";
    std::cout << "Principal Point         : " << rs_intr_color.ppx << ", " << rs_intr_color.ppy << std::endl;
    std::cout << "Focal Length            : " << rs_intr_color.fx << ", " << rs_intr_color.fy << std::endl;
    std::cout << "Distortion Model        : " << rs_intr_color.model << std::endl;
    std::cout << "Distortion Coefficients : [" << rs_intr_color.coeffs[0] << "," << rs_intr_color.coeffs[1] << "," <<
        rs_intr_color.coeffs[2] << "," << rs_intr_color.coeffs[3] << "," << rs_intr_color.coeffs[4] << "]" << std::endl;
  

    std::cout << "---------------Extrinsics Cam1---------------\n";
    std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
    std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
    std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
    std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;

    std::cout << "---------------Extrinsics Cam2---------------\n";
    std::cout << "Translation Vector : [" << extrinsics2.translation[0] << "," << extrinsics2.translation[1] << "," << extrinsics2.translation[2] << "]\n";
    std::cout << "Rotation Matrix    : [" << extrinsics2.rotation[0] << "," << extrinsics2.rotation[3] << "," << extrinsics2.rotation[6] << "]\n";
    std::cout << "                   : [" << extrinsics2.rotation[1] << "," << extrinsics2.rotation[4] << "," << extrinsics2.rotation[7] << "]\n";
    std::cout << "                   : [" << extrinsics2.rotation[2] << "," << extrinsics2.rotation[5] << "," << extrinsics2.rotation[8] << "]" << std::endl;
    std::cout << "---------------Extrinsics Depth---------------\n";
    std::cout << "Translation Vector : [" << extrinsics3.translation[0] << "," << extrinsics3.translation[1] << "," << extrinsics3.translation[2] << "]\n";
    std::cout << "Rotation Matrix    : [" << extrinsics3.rotation[0] << "," << extrinsics3.rotation[3] << "," << extrinsics3.rotation[6] << "]\n";
    std::cout << "                   : [" << extrinsics3.rotation[1] << "," << extrinsics3.rotation[4] << "," << extrinsics3.rotation[7] << "]\n";
    std::cout << "                   : [" << extrinsics3.rotation[2] << "," << extrinsics3.rotation[5] << "," << extrinsics3.rotation[8] << "]" << std::endl;
    std::cout << "---------------Extrinsics Color---------------\n";
    std::cout << "Translation Vector : [" << extrinsics4.translation[0] << "," << extrinsics4.translation[1] << "," << extrinsics4.translation[2] << "]\n";
    std::cout << "Rotation Matrix    : [" << extrinsics4.rotation[0] << "," << extrinsics4.rotation[3] << "," << extrinsics4.rotation[6] << "]\n";
    std::cout << "                   : [" << extrinsics4.rotation[1] << "," << extrinsics4.rotation[4] << "," << extrinsics4.rotation[7] << "]\n";
    std::cout << "                   : [" << extrinsics4.rotation[2] << "," << extrinsics4.rotation[5] << "," << extrinsics4.rotation[8] << "]" << std::endl;


    std::cout << "Closing Files..." << std::flush << std::endl;
    rec.close();
    std::cout << "Terminated." << std::flush << std::endl;
	return 0;
}catch(...)
{
    printf("Unhandled excepton occured'n");
    return EXIT_FAILURE;
}
