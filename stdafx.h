#pragma once

// Constants
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#ifndef PI
    #define PI 3.1415926535898
#endif

// C++ Libraries
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <climits>
#include <clocale>
#include <cfloat>
#include <cstdarg>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <deque>
#include <stack>
#include <thread>
#include <utility>
#include <algorithm>
#include <ctime>
#include <string>

// Boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

// OpenCV Libraries
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/features2d/features2d.hpp"

// PCL Libaries
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/impl/feature.hpp>
#include <pcl/point_traits.h>

// Eigen
#include <Eigen/SVD>

// Flann
#include <flann/util/serialization.h>

// LibSVM
#include "libSVM/svm.h"

// OpenARK namespace
namespace ark {
    typedef cv::Point Point;
    typedef cv::Point2i Point2i;
    typedef cv::Point2f Point2f;
    typedef cv::Point2d Point2d;
    typedef cv::Vec3b Vec3b;
    typedef cv::Vec3f Vec3f;
    typedef cv::Vec3d Vec3d;
    typedef cv::Vec3i Vec3i;
}