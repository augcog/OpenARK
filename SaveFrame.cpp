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
#include "Types.h"

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

    RGBDFrame SaveFrame::frameLoad(int frameId){
        std::cout<<"frameLoad start = "<< frameId <<std::endl;

		RGBDFrame frame;

        frame.frameId = frameId;


        frame.imRGB = cv::imread(rgbPath + std::to_string(frame.frameId) + ".jpg",cv::IMREAD_COLOR);

        cv::cvtColor(frame.imRGB, frame.imRGB, cv::COLOR_BGR2RGB);

        if(frame.imRGB.rows == 0){
            std::cout<<"frameLoad RGB fail = "<<frameId<<std::endl;
            frame.frameId = -1;
            return frame;
        }


        //if RGB images are not the same size as depth images
        //cv::resize(rgbBig, frame.imRGB, cv::Size(640,480));

        //rgbBig.release();
 
        frame.imDepth = cv::imread(depthPath + std::to_string(frame.frameId) + ".png",-1);

        if(frame.imDepth.rows == 0){
            std::cout<<"frameLoad depth fail = "<< frameId <<std::endl;
            frame.frameId = -1;
            return frame;
        }

        //depth255.convertTo(frame.imDepth, CV_32FC1);
		
        //depth255.release();

        //frame.imDepth *= 0.001;
        

        //TCW FROM XML
		std::ifstream file(tcwPath + std::to_string(frameId) + ".txt");
		for (int i = 0; i < 4; ++i) {
			for (int k = 0; k < 4; ++k) {
				file >> frame.mTcw.at<float>(i,k);
			}
		}
		file.close();



        if(frame.mTcw.rows == 0) {
            std::cout<<"frameLoad tcw fail = "<< frameId <<std::endl;
            frame.frameId = -1;
            return frame;
        }
        
        std::cout<<"frameLoad frame = "<< frameId <<std::endl;


        return frame;
    }

}
