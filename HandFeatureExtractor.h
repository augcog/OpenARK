#pragma once

#include "stdafx.h"
#include "Object3D.h"
#include "Util.h"

namespace classifier {
    namespace features {
        /**
        * Given a depth image, computes the mean and variance of the depth values and the distances from center.
        * For use with feature extraction.
        * @param xyz_map the depth image
        * @param center the centroid location in xyz coordinates
        * @param [out] avg_dist average distance from centroid
        * @param [out] var_dist variance of distances from centroid
        * @param [out] avg_depth average depth
        * @param [out] var_dist variance of depth
        */
        void computeMeanAndVariance(const cv::Mat& xyz_map, cv::Vec3f center,
            double& avg_dist, double& var_dist, double& avg_depth, double& var_depth);

        /** Extract finger tip and defect information from a given depth map of a hand.
         *  @param [in] depth the depth map
         *  @returns vector of features
         */
        std::vector<double> extractHandInfo(const cv::Mat & depth);

        /* Extract finger tip and defect information from a test case called "test_case_name" (please include leading zeros in name)
         *  in the data directory "data_dir". 
         *  @param test_case_name name of the test case
         *  @param data_dir path to the data directory
         *  @param depth_path optionally, a path to the depth image directory (inferred from data directory by default)
         *  @returns vector of features
         */
        std::vector<double> extractHandInfo(std::string test_case_name, std::string data_dir,
            std::string depth_path = "depth/");

        /** Extract hand-specific features from a given 3D object and depth map
         *  @param [in] obj Object3D instance
         *  @param [in] depth_map depth map (note: must be CV_32FC3)
         *  @param top_left optionally, top left point represented in depth map (x, y coordinates to translate by)
         *  @param img_scale optionally, amount the depth map has been scaled by
         *  @param full_wid optionally, size of full depth map. By default, uses width of depth_map
         *  @returns vector of features
         */
        std::vector<double> extractHandFeatures(Object3D & obj, const cv::Mat & depth_map,
            cv::Point top_left = cv::Point(0,0), double img_scale = 2.0, int full_wid = -1);

        /** Extract hand-specific features from a given depth map of a hand.
         *  @param [in] depth_map the depth map
         *  @returns vector of features
         */
        std::vector<double> extractHandFeatures(const cv::Mat & depth_map);

        /* Extract hand-specific features from a test case called "test_case_name" (please include leading zeros in name)
         *  in the data directory "data_dir".
         *  @param test_case_name name of the test case
         *  @param data_dir path to the data directory
         *  @param depth_path optionally, a path to the depth image directory (inferred from data directory by default)
         *  @returns vector of features
         */
        std::vector<double> extractHandFeatures(std::string test_case_name, std::string data_dir, std::string depth_path = "depth/");
    }
}
