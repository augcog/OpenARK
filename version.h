#pragma once

#define RSSDK_ENABLED
#define OPENARK_CAMERA_TYPE "sr300"
//#define PMDSDK_ENABLED
//#define OPENARK_CAMERA_TYPE "pmd" 

#define OPENARK_VERSION_MAJOR 0
#define OPENARK_VERSION_MINOR 9
#define OPENARK_VERSION_PATCH 2

// Uncomment to enable debug code
//#define DEBUG

// Necessary for typedefs
#include <opencv2/core/types.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <vector>

// Constants
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#ifndef PI
    #define PI 3.14159265358979323846
#endif

// OpenARK namespace
namespace ark {
    // OpenARK version number (modify in CMakeLists.txt)
    static const char * VERSION = "0.9.2";

    // Paths to possible SVM model file locations (terminated by "\n")
    static const char * SVM_PATHS[] = {"svm/", "../svm/", "", "\n"};

    // Typedefs for common types
    typedef cv::Point Point;
    typedef cv::Point2i Point2i;
    typedef cv::Point2f Point2f;
    typedef cv::Point2d Point2d;
    typedef cv::Vec2f Vec2f;
    typedef cv::Vec2d Vec2d;
    typedef cv::Vec2i Vec2i;
    typedef cv::Vec3b Vec3b;
    typedef cv::Vec3f Vec3f;
    typedef cv::Vec3d Vec3d;
    typedef cv::Vec3i Vec3i;

    // generic smart pointer shorthands
    typedef boost::shared_ptr<std::vector<Point2i> > VecP2iPtr;
    typedef boost::shared_ptr<std::vector<Vec3f> > VecV3fPtr;
}
