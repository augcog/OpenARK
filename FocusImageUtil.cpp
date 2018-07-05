#include "FocusImageUtil.h"

namespace ark
{
		FocusImageUtil::FocusImageUtil()
		{
			//Precompute maps for cv::remap()
			cv::initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
			cv::initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
		}

		//RemoveBackground
		void removeBackground(cv::Mat & image)
		{
			int iRows = image.rows;
			int iCols = image.cols;

			for (int i = 0; i < iRows; i++) {
				cv::Vec3f *p = image.ptr<cv::Vec3f>(i);

				for (int j = 0; j < iCols; j++) {
					if (p[j][2] > 0.5)
						p[j][2] = 0;
				}
			}
		}

		//Cut an image into 2 patch
		void FocusImageUtil::SplitImage(cv::Mat stereo_image, std::vector<cv::Mat>& ceil_image)
		{
			int rows = stereo_image.rows;
			int cols = stereo_image.cols / 2;

			for (int j = 0; j < 2; j++)
			{
				//Roi means "region of interest" algorithm
				cv::Mat roi;
				cv::Rect rect(j * cols, 0 * rows, cols, rows);
				stereo_image(rect).copyTo(roi);
				ceil_image.push_back(roi);
			}
		}
		void saveXYZ(const char* filename, const cv::Mat& mat)
		{
			const double max_z = 1.0e4;
			FILE* fp = fopen(filename, "wt");
			for (int y = 0; y < mat.cols; y++)
			{
				for (int x = 0; x < mat.rows; x++)
				{
					cv::Vec3f point = mat.at<cv::Vec3f>(x, y);
					if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
					fprintf(fp, "(%f %f %f) ", point[0], point[1], point[2]);
				}
				fprintf(fp, "\n/******************************************************/\n");
			}
			fclose(fp);
		}
		//Get depth mat using stereo_sgbm
		cv::Mat FocusImageUtil::SGBM_depth(cv::Mat img_l, cv::Mat img_r, int currFrame)
		{
			cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
<<<<<<< HEAD
			int numberOfDisparities = 48;
			int SADWindowSize = 2;
			numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_l.cols / 8) + 15) & -16;

			sgbm->setPreFilterCap(31);
=======
            cv::Mat cat;
            cv::vconcat(img_l, img_r, cat);
            cv::imshow("vcat", cat);
			int numberOfDisparities = 128;
			int SADWindowSize = 13;
			numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_l.cols / 8) + 15) & -16;

			sgbm->setPreFilterCap(73);
>>>>>>> 0494a82... Improved calibration; Adjusted detection parameters; Updated hand detection system to match newest OpenARK release
			int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
			sgbm->setBlockSize(sgbmWinSize);

			int cn = img_l.channels();

			sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
			sgbm->setP2(16 * cn*sgbmWinSize*sgbmWinSize);
			sgbm->setMinDisparity(0);
			sgbm->setNumDisparities(numberOfDisparities);
<<<<<<< HEAD
			sgbm->setUniquenessRatio(5);
			sgbm->setSpeckleWindowSize(10);
			sgbm->setSpeckleRange(1);
			sgbm->setDisp12MaxDiff(0);
=======
			sgbm->setUniquenessRatio(1);
			sgbm->setSpeckleWindowSize(100);
			sgbm->setSpeckleRange(1);//推荐值为1或2，原值为32
			sgbm->setDisp12MaxDiff(1);
>>>>>>> 0494a82... Improved calibration; Adjusted detection parameters; Updated hand detection system to match newest OpenARK release
			sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

			int min_disparity = sgbm->getMinDisparity();

			int bs2 = SADWindowSize / 2;
			int minD = min_disparity, maxD = min_disparity + numberOfDisparities - 1;

			int xmin = maxD + bs2;
			int xmax = img_l.size().width + minD - bs2;
			int ymin = bs2;
			int ymax = img_l.size().height - bs2;
			cv::Rect ROI(xmin, ymin, xmax - xmin, ymax - ymin);
			double lambda = 8000.0;
			double sigma = 1.7;

			cv::Mat disp;
			int64 t = cv::getTickCount();
			sgbm->compute(img_l, img_r, disp);
            /*
			cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

			wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
			wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*SADWindowSize));
			wls_filter->setLambda(lambda);
			wls_filter->setSigmaColor(sigma);
<<<<<<< HEAD
            */
            cv::Mat filtered_disp;
            //wls_filter->filter(disp, img_l, filtered_disp, cv::Mat(), ROI);
            filtered_disp = disp.clone();

            int ele_sz = 1;
            cv::Mat ele = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                cv::Size(2 * ele_sz + 1, 2 * ele_sz + 1), cv::Point(ele_sz, ele_sz));
            cv::dilate(filtered_disp, filtered_disp, ele);
            cv::erode(filtered_disp, filtered_disp, ele);
            cv::medianBlur(filtered_disp, filtered_disp, 3);

            // DEBUG 
            cv::Mat disp8, vcat1, vcat2, hcat;
=======
			cv::Mat filtered_disp;
            wls_filter->filter(disp, img_l, filtered_disp, cv::Mat(), ROI);
            //filtered_disp = disp.clone();


			//disp = getFilterDepth(img_l, img_r, sgbm, currFrame);
>>>>>>> 0494a82... Improved calibration; Adjusted detection parameters; Updated hand detection system to match newest OpenARK release
			filtered_disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
			
            cv::vconcat(img_l, img_r, vcat1);
            cv::vconcat(img_r, disp8, vcat2);
            cv::hconcat(vcat1, vcat2, hcat);
            cv::cvtColor(hcat, hcat, CV_GRAY2BGR);
            for (int i = 0; i < hcat.rows / 2; i += 20) {
                cv::line(hcat, cv::Point(0, i), cv::Point(hcat.cols, i), cv::Scalar(255, 0, 0));
            }
            cv::putText(hcat, "L", cv::Point(10, 25), 0, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            cv::putText(hcat, "R", cv::Point(hcat.cols - 30, 25), 0, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            cv::putText(hcat, "R", cv::Point(10, hcat.rows - 20), 0, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            cv::putText(hcat, "Disp", cv::Point(hcat.cols - 50, hcat.rows - 20), 0, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            cv::imshow("SGBM Debug", hcat);
            // END DEBUG 
            /*
			std::string filename = "C:\\Users\\vivedu\\Desktop\\img\\focus\\" + std::to_string(currFrame) + ".jpg";
			cv::imwrite(filename, disp8);
            */
			cv::Mat out;
        	////cv::reprojectImageTo3D(disp, out, Q, true);
			cv::reprojectImageTo3D(filtered_disp, out, Q, true);
            /*
            double avg = 0.0;
            int cnt = 0;
            for (int i = 0; i < out.rows; ++i) {
                for (int j = 0; j < out.cols; ++j) {
                    auto v = out.at<cv::Vec3f>(i, j);
                    if (v[2] < 1000 && v[2] > 0) {
                        cnt++;
                        avg += v[2];
                    }
                }
            }
            if (cnt) std::cout << avg / cnt << "\n";
            */
<<<<<<< HEAD
			out *= 18;
=======
			out *= 10;
>>>>>>> 0494a82... Improved calibration; Adjusted detection parameters; Updated hand detection system to match newest OpenARK release

			//remove background from out image
			//removeBackground(out);

			//convert disp(CV_16S)  to ark xyzmap(CV_32FC3)
			//disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
			//disp8.convertTo(disp32, CV_32FC1, 1 / 255.0);
			//cv::Mat out;
			//cvtColor(disp32, out, cv::COLOR_GRAY2BGR);

			t = cv::getTickCount() - t;
			//std::cout << "SGBM algorithm Time elapsed:" << t * 1000 / cv::getTickFrequency() << std::endl;

			//saveXYZ("C:/Users/vivedu/Desktop/img/xyzmap.xml", out);
			return out;
		}

		//Get depth mat using stereo_bm
		cv::Mat FocusImageUtil::BM_depth(cv::Mat img_l, cv::Mat img_r)
		{
			int numberOfDisparities = 96;
			cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
			cv::Rect roi1, roi2;
			bm->setROI1(roi1);
			bm->setROI2(roi2);
			bm->setPreFilterCap(31);
			bm->setBlockSize(7);
			bm->setMinDisparity(0);
			bm->setNumDisparities(numberOfDisparities);
			bm->setTextureThreshold(10);
			bm->setUniquenessRatio(15);
			bm->setSpeckleWindowSize(100);
			bm->setSpeckleRange(1);
			bm->setDisp12MaxDiff(1);

			cv::Mat disp, disp8, disp32;
			int64 t = cv::getTickCount();
			disp = getFilterDepth(img_l, img_r, bm);
			t = cv::getTickCount() - t;
			//std::cout << "BM algorithm Time elapsed:" << t * 1000 / cv::getTickFrequency() << std::endl;
			//convert disp(CV_16S)  to ark xyzmap(CV_32FC3)
			disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
			disp8.convertTo(disp32, CV_32FC1, 1 / 255.0);
			cv::Mat out;
			cv::reprojectImageTo3D(disp, out, Q, true);
            //out /= 100;

			//remove background from out image
			removeBackground(out);

			return out;
		}

		//Calibrate the image
		void FocusImageUtil::CalibrateImage(cv::Mat img_l, cv::Mat img_r, cv::Mat &calibrated_img_l, cv::Mat &calibrated_img_r)
		{
			cv::remap(img_l, calibrated_img_l, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
			cv::remap(img_r, calibrated_img_r, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
		}

		//Get image_left_calibrated from focusImageMat
		cv::Mat FocusImageUtil::get_image_left_calibrated(cv::Mat focusImageMat,int currFrame)
		{
			std::vector<cv::Mat> imageList;
			SplitImage(focusImageMat, imageList);
			cv::Mat image_left_calibrated, image_right_calibrated;
			CalibrateImage(imageList[0], imageList[1], image_left_calibrated, image_right_calibrated);
			return image_left_calibrated;
		}


		//Get depth mat from focus image mat
		cv::Mat FocusImageUtil::GetDepthMap(cv::Mat focusImageMat, int algorithm, int currFrame)
		{
			std::vector<cv::Mat> imageList;
			SplitImage(focusImageMat, imageList);
			cv::Mat image_left_calibrated, image_right_calibrated;
			CalibrateImage(imageList[0], imageList[1], image_left_calibrated, image_right_calibrated);
			//debug
			if(algorithm == STEREO_BM)
				return BM_depth(image_left_calibrated, image_right_calibrated);
			else
				return SGBM_depth(image_left_calibrated, image_right_calibrated, currFrame);
		}

		// Get filtered Depth from StereoMatcher
		cv::Mat FocusImageUtil::getFilterDepth(cv::Mat img_l, cv::Mat img_r, cv::Ptr<cv::StereoMatcher> left_matcher, int currFrame)
		{
			int numberOfDisparities = 96;
			cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
			cv::Mat disp_left, disp8, disp_right, filtered_disp;

			left_matcher->compute(img_l, img_r, disp_left);
			right_matcher->compute(img_r, img_l, disp_right);
			cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
			wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
			double lambda = 8000.0;
			double sigma = 1.7;

			wls_filter->setLambda(lambda);
			wls_filter->setSigmaColor(sigma);
			wls_filter->filter(disp_left, img_l, filtered_disp, disp_right);

			return filtered_disp;
		}
}
