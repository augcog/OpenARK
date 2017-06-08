#pragma once
//Intel RealSense 3D SDK libraries
#include "RealSense/SenseManager.h"
#include "RealSense/SampleReader.h"
#include "RealSense/Session.h"

//OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

/**
* Class for converting Intel RealSense images to OpenCV image format
*/
class Converter
{
public:

	/**
	* Converting an Intel RealSense 3D camera image to OpenCV image format
	* @param inImg input image
	* @param data Intel RealSense image data
	* @param outImg output image in form of OpenCV image
	*/
	static void Converter::ConvertPXCImageToOpenCVMat(Intel::RealSense::Image *inImg, Intel::RealSense::ImageData data, cv::Mat *outImg);
};