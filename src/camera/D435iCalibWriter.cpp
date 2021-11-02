
#include <librealsense2/rs.hpp> 
#include <Eigen/Core>   
#include <iostream>
#include "camera/D435iCalibWriter.h"


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
