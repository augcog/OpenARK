#define NOMINMAX
#define _WINSOCKAPI_  

#pragma once
#include <opencv/cxcore.h>

#include "RealSense/SenseManager.h"
#include "RealSense/SampleReader.h"

#include "RealSense/Session.h"
#include <opencv2/opencv.hpp>

#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

class Converter
{
public:
	static void Converter::ConvertPXCImageToOpenCVMat(Intel::RealSense::Image *inImg, Intel::RealSense::ImageData data, cv::Mat *outImg);
};