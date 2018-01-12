#pragma once

#include "stdafx.h"

//Intel RealSense 3D SDK libraries
#include "RealSense/SampleReader.h"
#include "RealSense/Session.h"

namespace ark {

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
}