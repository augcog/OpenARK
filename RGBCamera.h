#pragma once
//OpenCV libraries
#include "opencv2/highgui/highgui.hpp"

/**
* Abstract class that defines the behavior of a RGB camera.
*/
class RGBCamera
{
public:
	/**
	* Updates the current frame on the RGB camera.
	* Should be overriden by a concerte implementation specific to the RGB camera
	*/
	virtual void update() = 0;

	/**
	* Returns the current frame.
	* @return the current frame
	*/
	cv::Mat getFrame();

protected:
	/**
	* Camera handle.
	*/
	cv::VideoCapture cap;

	/**
	* Current frame.
	*/
	cv::Mat frame;
};