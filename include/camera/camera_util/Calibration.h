#pragma once

#include "Version.h"

#include <vector>
#include <string>
#include <opencv2/core.hpp>

// OpenARK Libraries
#include "util/Visualizer.h"
#include "DepthCamera.h"
#include "RGBCamera.h"
#include "util/Util.h"

namespace ark {
    /**
    * Class for various calibration operations.
    */
    class Calibration
    {
    public:
        /**
        * Compute a calibration from (x,y,z) real world coordinates to (x',y',z') Unity coordinates.
        * @param depth_cam instance of a live @see DepthCamera
        * @param num_boards number of checkboard positions
        * @param board_w width of the board (number of inner intersections)
        * @param board_h height of the board (number of inner intersections)
        */
        static void XYZToUnity(DepthCamera& depth_cam, int num_boards, int board_w, int board_h);

        /**
        * Compute a calibration from (x,y,z) real world coordiantes to (i,j) RGB camera coordinates.
        * @param depth_cam instance of a live @see DepthCamera
        * @param rgb_cam instance of a live @see RGBCamera
        * @param num_boards number of checkboard positions
        * @param board_w width of the board (number of inner intersections)
        * @param board_h height of the board (number of inner intersections)
        */
        static void XYZToRGB(DepthCamera* depth_cam, RGBCamera* rgb_cam, int num_boards, int board_w, int board_h);

        /**
        * Reproject the (x,y,z) points to (x',y',z') Unity points with the RT matrix to test for error.
        * @param XYZ_points coordinates from the PMD sensor
        * @param Unity_points coordinates from the game engine
        * @param R the rotation matrix to use
        * @param T the translation matrix to use
        * @return cummulative reprojection error
        */
        static double reprojectXYZToUnity(std::vector<std::vector<Vec3f>> XYZ_points, std::vector<std::vector<Vec3f>> Unity_points, Eigen::MatrixXf R, Eigen::MatrixXf T);

        /**
        * Reproject the (x,y,z) points to (i,j) RGB image points with the RT matrix to test for error.
        */
        static double reprojectXYZtoRGB();

        /**
        * Compute full set of Unity checkboard coordinates.
        * @param upper_left the upper left corner coordinates of the chessboard
        * @param distance size of the checkerboard (meters)
        * @param num_rows number of rows (inner intersections)
        * @param num_cols number of columns (inner intersections)
        */
        static std::vector<std::vector<Vec3f>> prepareUnityData(std::vector<Vec3f> upper_left, float distance, int num_rows, int num_cols);

        /**
        * Write calibration data points to file
        * @param points data points to be written to file
        * @param board_w board width (used for formatting the file)
        * @param board_h board height (used for formatting the file)
        * @param filename path of the file
        */
        static void writeDataToFile(std::vector<std::vector<Vec3f>> points, int board_w, int board_h, std::string filename);

    private:
        /**
        * Compute the translation and rotation matrix from coordinate system x->y given a set of corresponding points
        * @param [in] x set of coordinates from the first coordinate system
        * @param [in] y set of corresponding coordinates from the second coordinate system
        * @param [out] R resultant rotation matrix
        * @param [out] t resultant translation matrix
        */
        static void computeRT(cv::Mat x, cv::Mat y, cv::Mat *R, cv::Mat *t);

    };
}
