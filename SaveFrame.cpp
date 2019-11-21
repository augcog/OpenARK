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

		frame_counter = 15;

		//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

    }

    void SaveFrame::frameWrite(cv::Mat imRGB, cv::Mat depth, Eigen::Matrix4d traj, int frameId){

		frame_counter++;

		if (frame_counter > 2) {
			frame_counter = 0;
		}
		else {
			return;
		}

        std::cout<<"frameWrite frame = "<< frameId <<std::endl;

		cv::Mat imBGR;
        cv::cvtColor(imRGB, imBGR, CV_RGB2BGR);
        cv::imwrite(rgbPath + std::to_string(frameId) + ".jpg", imBGR);

		//std::cout << "rgb done" << std::endl;



		//cv::imshow("Display window", depth);
		//cv::waitKey(1);
		


		//cv::Mat depth255;
        //depth.convertTo(depth255, CV_16UC1);

		//cv::imshow("Display window 2", depth255);
		//cv::waitKey(1);

        cv::imwrite(depthPath + std::to_string(frameId) + ".png", depth);

		//cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
		//fs << "depth" << depth255;
		//fs.release();

		//std::cout << "depth done" << std::endl;

		//cv::Mat transform = cv::Mat(cv::Size(4, 4), CV_64FC1);

		//cv::eigen2cv(traj, transform);

		std::ofstream file(tcwPath + std::to_string(frameId) + ".txt");
		if (file.is_open())
		{
			file << traj.matrix() << '\n';
		}
		file.close();


        //TCW is World to Camera Transform

		/*cv::Mat transform  = cv::Mat(4, 4, CV_32FC1);
		cv::OutputArray output = cv::OutputArray(transform);

		std::cout << "here2" << std::endl;

		cv::eigen2cv(traj, output);
		
		std::cout << "here1" << std::endl;

        cv::FileStorage fs(tcwPath + std::to_string(frameId)+".xml",cv::FileStorage::WRITE);
        fs << "tcw" << transform;
        //fs << "depth" << frame.imDepth ;
        fs.release();*/

        //std::cout << "finished writing " << frameId << std::endl;



        //RGB and Depth to .xml (.png preferable)
        /*
        cv::FileStorage fs2(depth_to_tcw_Path + std::to_string(frame.frameId)+".xml",cv::FileStorage::WRITE);
        fs2 << "depth" << depth255;
        // fs << "rgb" << frame.imRGB;
        fs2.release();
        */
		

    }

    /*RGBDFrame SaveFrame::frameLoad(int frameId){
        std::cout<<"frameLoad start = "<< frameId <<std::endl;

        RGBDFrame frame;

        frame.frameId = frameId;


        frame.imRGB = cv::imread(rgbPath + std::to_string(frame.frameId) + ".png",cv::IMREAD_COLOR);

        cv::cvtColor(frame.imRGB, frame.imRGB, cv::COLOR_BGR2RGB);

        if(frame.imRGB.rows == 0){
            std::cout<<"frameLoad RGB fail = "<<frameId<<std::endl;
            frame.frameId = -1;
            return frame;
        }


        //if RGB images are not the same size as depth images
        //cv::resize(rgbBig, frame.imRGB, cv::Size(640,480));

        //rgbBig.release();
 
        cv::Mat depth255 = cv::imread(depthPath + std::to_string(frame.frameId) + ".png",-1);

        if(depth255.rows == 0){
            std::cout<<"frameLoad depth fail = "<< frameId <<std::endl;
            frame.frameId = -1;
            return frame;
        }

        depth255.convertTo(frame.imDepth, CV_32FC1);

        depth255.release();

        frame.imDepth *= 0.001;
        

        //TCW FROM XML
        cv::FileStorage fs2(tcwPath + std::to_string(frame.frameId) + ".xml", cv::FileStorage::READ);
        fs2["tcw"] >> frame.mTcw;
        fs2.release();



        if(frame.mTcw.rows == 0) {
            std::cout<<"frameLoad tcw fail = "<< frameId <<std::endl;
            frame.frameId = -1;
            return frame;
        }

        
        std::cout<<"frameLoad frame = "<< frameId <<std::endl;


        return frame;
    }*/

}
