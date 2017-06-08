#pragma once
// C++ Libraries
#include<string.h>

// PMD Libraries
#include <pmdsdk2.h>

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
* Class defining the behavior of a PMD Camera.
* Example on how to read from sensor and visualize its output
* @include SensorIO.cpp
*/
class PMDCamera : public DepthCamera
{
public:
	/**
	* Public constructor initializing the PMD Camera.
	* @param use_live_sensor uses input from real sensor if TRUE. Otherwise reads from input file. Default is set to TRUE.
	*/
	PMDCamera(bool use_live_sensor = true);

	/**
	* Deconstructor for the PMD Camera.
	*/
	~PMDCamera();

	/**
	* Gets new frame from sensor.
	* Updates xyzMap, ampMap, and flagMap. Resets clusters.
	*/
	void update();

	/**
	* Gracefully closes the PMD camera.
	*/
	void destroyInstance();

private:
	/**
	* Getter method for the x-coordinate at (i,j).
	* @param i ith row
	* @param j jth column
	* @return x-coodinate at (i,j)
	*/
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
	void fillInZCoords();

	/**
	* Update the values in the ampMap.
	*/
	void fillInAmps();

	//Private Variable
	const char* SOURCE_PLUGIN = "camboardpico";
	const char* SOURCE_PARAM = "";
	const char* PROC_PLUGIN = "camboardpicoproc";
	const char* PROC_PARAM = "";

	PMDHandle hnd;
	PMDDataDescription dd;
	char err[128]; // Char array for storing PMD's error log

	int numPixels;
	float* dists;
	float* amps;

	cv::Mat frame;
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
};

/*
* \include SensorIO.cpp
* Example of how to read from sensor
*/