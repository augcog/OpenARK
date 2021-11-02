#include "stdafx.h"
#include "Version.h"
#include "camera/StereoCamera.h"
#include "util/Visualizer.h"


namespace ark {
    StereoCamera::StereoCamera(StereoCalibration::Ptr calib, std::function<cv::Mat(void)> imageSource, SGBMConfig::Ptr sgbmConf)
        : StereoCamera(calib, sgbmConf) { 
        imageSource = imageSource;
    }

    StereoCamera::StereoCamera(StereoCalibration::Ptr calib, SGBMConfig::Ptr sgbmConf)
    {
        setCalibration(calib);
        this->sgbmConf = sgbmConf ? sgbmConf : std::make_shared<SGBMConfig>();
    }

    // overrided model name
    const std::string StereoCamera::getModelName() const
    {
        return "Stereo";
    }

    // overrided width
    int StereoCamera::getWidth() const {
        return calib->imageSize.width;
    }

    // overrided height
    int StereoCamera::getHeight() const {
        return calib->imageSize.height;
    }

    const DetectionParams::Ptr & StereoCamera::getDefaultParams() const
    {
        static DetectionParams::Ptr params = DetectionParams::create(); // default parameters
        if (params->handClusterInterval != 4) {
            params->handMaxDepth = 1.200;
            params->handClusterMaxDistance = 0.000034;
            params->handMinArea = 0.012;
            params->handMaxArea = 0.080;
            params->wristWidthMax = 0.10;
            params->wristWidthMin = 0.015;
            params->wristCenterDistThresh = 0.1;
            params->handClusterMinPoints = 0.0167;
            params->handClusterInterval = 4;
            params->xyzAverageSize = 6;
            params->handSVMConfidenceThresh = 0.2;
            params->contourImageErodeAmount = 1;
            params->contourImageDilateAmount = 1;
            params->centerMaxDistFromTop = 0.2;
            params->fingerLenMin = 0.051;
            params->centroidDefectFingerAngleMin = 0.25 * PI;
        }
        return params;
    }

    /**
    * true if has RGB image (override)
    */
    bool StereoCamera::hasRGBMap() const {
        return true;
    }

    /**
    * Create xyzMap, zMap, ampMap, and flagMap from sensor input (override)
    * @param [out] xyz_map XYZ map (projection point cloud). CV_32FC3
    * @param [out] rgb_map RGB image. CV_8UC3 (NOT USED)
    * @param [out] ir_map IR image. CV_8UC1
    * @param [out] amp_map amplitude map. CV_32FC1 (NOT USED)
    * @param [out] flag_map flag map. CV_8UC1 (NOT USED)
    */
    void StereoCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                             cv::Mat & amp_map, cv::Mat & flag_map) 
    {
        rgb_map = imageSource();
        xyz_map = computeDepthSGBM(rgb_map);
    }

    void StereoCamera::setCalibration(StereoCalibration::Ptr calib)
    {
        this->calib = calib;
        // precompute maps for cv::remap()
        cv::initUndistortRectifyMap(calib->cameraMatrix1, calib->distCoeffs1, calib->R1, calib->P1, calib->undistortedSize, CV_16SC2, rmap[0][0], rmap[0][1]);
        cv::initUndistortRectifyMap(calib->cameraMatrix2, calib->distCoeffs2, calib->R2, calib->P2, calib->undistortedSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    }

    void StereoCamera::setSGBMConfig(SGBMConfig::Ptr sgbmConf)
    {
        this->sgbmConf = sgbmConf;
    }

    // Calibrate the image
    void StereoCamera::calibrateImage(cv::Mat img_l, cv::Mat img_r, cv::Mat & calibrated_img_l, cv::Mat & calibrated_img_r)
    {
        cv::remap(img_l, calibrated_img_l, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
        cv::remap(img_r, calibrated_img_r, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
    }

    // Get depth mat using stereo_sgbm
    cv::Mat StereoCamera::computeDepthSGBM(cv::Mat frame, cv::Mat * left_calibrated)
    {
        cv::Mat img_l_raw, img_r_raw, img_l, img_r;
        splitImage(frame, img_l_raw, img_r_raw);
        calibrateImage(img_l_raw, img_r_raw, img_l, img_r);

        if (left_calibrated) *left_calibrated = img_l;

        int sgbmWinSize = sgbmConf->windowSize;
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, sgbmConf->disparities, sgbmWinSize);
        sgbm->setPreFilterCap(sgbmConf->preFilterCap);
        sgbm->setBlockSize(sgbmConf->windowSize);

        int cn = img_l.channels();
        sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(16 * cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(sgbmConf->minDisparity);
        sgbm->setNumDisparities(sgbmConf->disparities);
        sgbm->setUniquenessRatio(sgbmConf->uniquenessRatio);
        sgbm->setSpeckleWindowSize(sgbmConf->speckleWindowSize);
        sgbm->setSpeckleRange(sgbmConf->speckleRange);
        sgbm->setDisp12MaxDiff(sgbmConf->dispL2MaxDiff);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

        cv::Mat disp;
        sgbm->compute(img_l, img_r, disp);

        int ele_sz = sgbmConf->erodeDilateSize;
        cv::Mat ele = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(2 * ele_sz + 1, 2 * ele_sz + 1), cv::Point(ele_sz, ele_sz));
        cv::dilate(disp, disp, ele);
        cv::erode(disp, disp, ele);
        cv::medianBlur(disp, disp, sgbmConf->medianBlurSize);

#ifdef DEBUG
        cv::Mat disp8, vcat1, vcat2, hcat;
        disp.convertTo(disp8, CV_8U, 255 / (sgbmConf->disparities *16.));

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
#endif
        cv::Mat out;
        cv::reprojectImageTo3D(disp, out, calib->Q, true);
        out *= sgbmConf->scaleAmount;
        return out;
    }

    void StereoCamera::splitImage(cv::Mat hcat, cv::Mat & img_l, cv::Mat & img_r)
    {
        int wid = hcat.cols / 2, hi = hcat.rows;
        hcat(cv::Rect(0, 0, wid, hi)).copyTo(img_l);
        hcat(cv::Rect(wid, 0, wid, hi)).copyTo(img_r);
    }

    // get image_left_calibrated from focusImageMat
    cv::Mat StereoCamera::computeImageLeftCalibrated(cv::Mat focusImageMat)
    {
        cv::Mat img_l_raw, img_r_raw;
        splitImage(focusImageMat, img_l_raw, img_r_raw);
        cv::Mat img_l, img_r;
        calibrateImage(img_l_raw, img_r_raw, img_l, img_r);
        return img_l;
    }

    StereoCalibration::StereoCalibration(const std::string & yaml_file)
    {
        load(yaml_file);
    }

    void StereoCalibration::load(const std::string & yaml_file)
    {
        cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
        fs["imageWidth"] >> imageSize.width;
        fs["imageHeight"] >> imageSize.height;
        fs["undistortedWidth"] >> undistortedSize.width;
        fs["undistortedHeight"] >> undistortedSize.height;
        fs["cameraMatrix1"] >> cameraMatrix1;
        fs["cameraMatrix2"] >> cameraMatrix2;
        fs["distCoeffs1"] >> distCoeffs1;
        fs["distCoeffs2"] >> distCoeffs2;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
        fs["Q"] >> Q;
        fs.release();
    }

    void StereoCalibration::save(const std::string & yaml_file) const
    {
        cv::FileStorage fs(yaml_file, cv::FileStorage::WRITE);
        fs << "imageWidth" << imageSize.width;
        fs << "imageHeight" << imageSize.height;
        fs << "cameraMatrix1" << cameraMatrix1;
        fs << "cameraMatrix2" << cameraMatrix2;
        fs << "distCoeffs1" << distCoeffs1;
        fs << "distCoeffs2" << distCoeffs2;
        fs << "R1" << R1;
        fs << "R2" << R2;
        fs << "P1" << P1;
        fs << "P2" << P2;
        fs << "Q" << Q;
        fs.release();
    }
}

