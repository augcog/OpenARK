#pragma once
#include "opencv2/ximgproc.hpp"

namespace ark
{
	class FocusImageUtil
	{
	public:
		FocusImageUtil();
		enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };

		//Cut an image into 2 patch
		void SplitImage(cv::Mat stereo_image, std::vector<cv::Mat>& ceil_image);
		//Get depth mat using stereo_sgbm
		cv::Mat SGBM_depth(cv::Mat img_l, cv::Mat img_r, int currFrame = 0);

		//Get depth mat using stereo_bm
		cv::Mat BM_depth(cv::Mat img_l, cv::Mat img_r);

		//Calibrate the image
		void CalibrateImage(cv::Mat img_l, cv::Mat img_r, cv::Mat &calibrated_img_l, cv::Mat &calibrated_img_r);

		//Get depth mat from focus image mat
		cv::Mat GetDepthMap(cv::Mat focusImageMat, int algorithm = STEREO_SGBM, int currFrame = 0);

		// Get filtered Depth from StereoMatcher
		cv::Mat FocusImageUtil::getFilterDepth(cv::Mat img_l, cv::Mat img_r, cv::Ptr<cv::StereoMatcher> left_matcher, int currFrame = 0);

		//Get image_left_calibrated from focusImageMat
		cv::Mat FocusImageUtil::get_image_left_calibrated(cv::Mat focusImageMat, int currFrame);
		
		/*
*/
	private:
		cv::Mat cameraMatrix_left = (cv::Mat_<double>(3, 3) << 290.06063995171763, 0., 336.43453994930604,
            0., 289.8075842725341, 197.11087816249486, 0., 0., 1.);
		cv::Mat cameraMatrix_right = (cv::Mat_<double>(3, 3) << 290.26941994829207, 0., 332.8637318526213,
            0., 289.7640646306046, 194.38153983374764, 0., 0., 1.);
        cv::Mat distCoeffs_left = (cv::Mat_<double>(1, 5) <<
            -0.24318429366374802, 0.04303410396227692, 0.003317736556228473,
            -0.004101904125909965, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.);
		cv::Mat distCoeffs_right = (cv::Mat_<double>(1, 5) <<
            -0.24972676441551098, 0.046695062184188736, 0.0019932783249151983,
            -0.0030637841991412494, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.);

		cv::Mat R1 = (cv::Mat_<double>(3, 3) << 0.9953701300521662, -0.002956500796288157, 0.09607061623085122,
            0.0024758109321807503, 0.9999838159542164, 0.005122322702834883,
            -0.09608420557074945, -0.004860754332964242, 0.9953613406734022);
		cv::Mat R2 = (cv::Mat_<double>(3, 3) << 0.9967764606328982, -0.0017899769367439398, 0.08020899893851374,
            0.002207836959059592, 0.9999844489760743, -0.005121255898617975,
            -0.0801985846765065, 0.005281835720925395, 0.9967649117155472);
		cv::Mat P1 = (cv::Mat_<double>(3, 4) << 289.78582445156934, 0., 236.53733825683594,
            0., 0., 289.78582445156934,
            198.94229125976562, 0., 0., 0., 1., 0.);
		cv::Mat P2 = (cv::Mat_<double>(3, 4) << 289.78582445156934, 0., 236.53733825683594,
            -18.7797732450253, 0., 289.78582445156934,
            198.94229125976562, 0., 0., 0., 1., 0.);
        /*
		cv::Mat Q = (cv::Mat_<double>(4, 4) << 1., 0., 0., -243.25770568847656,
            0., 1., 0., -200.6678066253662,
            0., 0., 0., 291.542424289802,
            0., 0., 0.7707851300488535, -0.);
*/
        cv::Mat Q = (cv::Mat_<double>(4, 4) << 1., 0., 0.,
            -236.53733825683594, 0., 1.,
            0., -198.94229125976562, 0.,
            0., 0., 289.78582445156934,
            0., 0., 15.43074139770738, -0.);

		cv::Mat rmap[2][2];
		cv::Size imageSize = cv::Size(480, 300);
	};
}