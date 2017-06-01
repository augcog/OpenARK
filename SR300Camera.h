#define NOMINMAX
#define _WINSOCKAPI_  

#pragma once
// C++ Libraries
#include<string.h>

//realsense library 
#include "RealSense/SenseManager.h"
#include "RealSense/SampleReader.h"
#include "RealSense/Session.h"
#include <opencv2/opencv.hpp>
// OpenCV Libraries
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>


// OpenARK Libraries
#include "DepthCamera.h"

/**
* Class defining the behavior of an SR300 Camera.
* Example on how to read from sensor and visualize its output
* @include SensorIO.cpp
*/
class SR300Camera : public DepthCamera
{
public:

	

	/**
	* Public constructor initializing the SR300 Camera.
	* @param use_live_sensor uses input from real sensor if TRUE. Otherwise reads from input file. Default is set to TRUE.
	*/


	SR300Camera(bool use_live_sensor = true);

	/**
	* Deconstructor for the SR300 Camera.
	*/
	~SR300Camera();



	/**
	* Gets new frame from sensor.
	* Updates xyzMap, ampMap, and flagMap. Resets clusters.
	*/
	void update();

	/**
	* Gracefully closes the SR300 camera.
	*/
	void destroyInstance();

private:
	/**
	* Getter method for the x-coordinate at (i,j).
	* @param i ith row
	* @param j jth column
	* @return x-coodinate at (i,j)
	*/
	//void getXYZbuffer();

	float getX(int i, int j) const;


	/**
	* Getter method for the x-coordinate at (i,j).
	* @param i ith row
	* @param j jth column
	* @return x-coodinate at (i,j)
	*/
	float getY(int i, int j) const;

	/**
	* Getter method for the x-coordinate at (i,j).
	* @param i ith row
	* @param j jth column
	* @return x-coodinate at (i,j)
	*/
	float getZ(int i, int j) const;

	/**
	* Update the z-coordinates of the xyzMap.
	*/
//	void fillInZCoords();

	/**
	* Update the values in the ampMap.
	*/
	void fillInAmps();

	void fillInZCoords();


	/**
	* Convert the depth coordinates to world coordinates
	*/
	//void DepthToWorld(Image *depth, vector<PointF32> dcords, vector<PointF32> &wcords) const;



	//Private Variable
	const char* SOURCE_PLUGIN = "camboardpico";
	const char* SOURCE_PARAM = "";
	const char* PROC_PLUGIN = "camboardpicoproc";
	const char* PROC_PARAM = "";

	//SR300handle hnd;
	//SR300DataDescription dd;
	char err[128]; // Char array for storing PMD's error log

	int numPixels;
	float* dists;
	float* amps;

	//IplImage * frame;
	cv::Mat frame; 
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;



};




/*
* \include SensorIO.cpp
* Example of how to read from sensor
*/