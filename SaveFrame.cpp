//
// Created by yiwen on 2/2/19.
//
/*
Esther commented
- Enables online frame writing to folder for offline reconstruction later
- Enables offline file access
- Requires onKeyFrameAvailable handler in SLAM
- Used in rgbd_realsense_d435, rgbd_realsense_load_gl, rgbd_realsense_load_categorized
- TCW from ORBSLAM is World to Camera Transform.
*/

#include <chrono>
#include <mutex>
#include <iostream>
#include <fstream>
#include <direct.h>

//#include <MathUtils.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/fast_bilateral.h>
// #include <opencv2/ximgproc.hpp>
#include <opencv2/opencv.hpp>
#include "SaveFrame.h"

namespace ark {

    void createFolder(std::string folderPath){
		mkdir(folderPath.c_str());
		std::cout << folderPath << "dir made" << std::endl;
    }


    SaveFrame::SaveFrame(std::string folderPath) {

        createFolder(folderPath);

        rgbPath = folderPath +"RGB/";
        depthPath = folderPath +"depth/";
        tcwPath = folderPath +"tcw/";

        createFolder(rgbPath);
        createFolder(depthPath);
        createFolder(tcwPath);

    }

    void SaveFrame::frameWrite(cv::Mat imRGB, cv::Mat depth, Eigen::Matrix4d traj, int frameId){

        std::cout<<"frameWrite frame = "<< frameId <<std::endl;

		cv::Mat imBGR;
        cv::cvtColor(imRGB, imBGR, CV_RGB2BGR);
        cv::imwrite(rgbPath + std::to_string(frameId) + ".jpg", imBGR);

        cv::imwrite(depthPath + std::to_string(frameId) + ".png", depth);

		std::ofstream file(tcwPath + std::to_string(frameId) + ".txt");
		if (file.is_open())
		{
			file << traj.matrix() << '\n';
		}
		file.close();
    }

}
