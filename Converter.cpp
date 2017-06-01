#define NOMINMAX
#define _WINSOCKAPI_  

#include "Converter.h"



/***
Returns the next frame if next frame is recorded
Returns the previous frame if next frame is not recorded
***/
void Converter::ConvertPXCImageToOpenCVMat(Intel::RealSense::Image *inImg, Intel::RealSense::ImageData data, cv::Mat *outImg){

	int cvDataType;
	int cvDataWidth;

	Intel::RealSense::Image::ImageInfo imgInfo = inImg->QueryInfo();

	switch (data.format) {

	case Intel::RealSense::Image::PIXEL_FORMAT_YUY2:
	case Intel::RealSense::Image::PIXEL_FORMAT_NV12:
		throw(0); // Not implemented
	case Intel::RealSense::Image::PIXEL_FORMAT_RGB32:
		cvDataType = CV_8UC4;
		cvDataWidth = 4;
		break;
	case Intel::RealSense::Image::PIXEL_FORMAT_RGB24:
		cvDataType = CV_8UC3;
		cvDataWidth = 3;
		break;
	case Intel::RealSense::Image::PIXEL_FORMAT_Y8:
		cvDataType = CV_8U;
		cvDataWidth = 1;
		break;


	case Intel::RealSense::Image::PIXEL_FORMAT_DEPTH:
	case Intel::RealSense::Image::PIXEL_FORMAT_DEPTH_RAW:
		cvDataType = CV_16U;
		cvDataWidth = 2;
		break;
	case Intel::RealSense::Image::PIXEL_FORMAT_DEPTH_F32:
		cvDataType = CV_32F;
		cvDataWidth = 4;
		break;


	case Intel::RealSense::Image::PIXEL_FORMAT_Y16:
		cvDataType = CV_16U;
		cvDataWidth = 2;
		break;
	case Intel::RealSense::Image::PIXEL_FORMAT_Y8_IR_RELATIVE:
		cvDataType = CV_8U;
		cvDataWidth = 1;
		break;
	}

	// suppose that no other planes
	if (data.planes[1] != NULL) throw(0); // not implemented
										  // suppose that no sub pixel padding needed
	if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented

	outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
	
	//memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
	memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(uint8_t));
}

