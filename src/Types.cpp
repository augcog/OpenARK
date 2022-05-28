# include "openark/util/Types.h"

namespace ark {

    bool ImuMeasurement::operator<(const ImuMeasurement& right) const
    {
        return timestamp > right.timestamp;
    }

    RGBDFrame::RGBDFrame()
    {
        mTcw = cv::Mat::eye(4, 4, CV_32FC1);
        frameId = -1;
    }

    RGBDFrame::RGBDFrame(const RGBDFrame& frame)
    {
        frame.mTcw.copyTo(mTcw);
        frame.imRGB.copyTo(imRGB);
        frame.imDepth.copyTo(imDepth);
        frameId = frame.frameId;
    }

    CameraCalibration::CameraCalibration(double fx, double fy, double cx, double cy, cv::Mat distortion_coeffs /*= cv::Mat()*/) :
        distortion_coeffs(distortion_coeffs)
    {
        camera_matrix = cv::Mat(3, 3, cv::DataType<double>::type);
        camera_matrix.at<double>(0, 0) = fx;
        camera_matrix.at<double>(1, 1) = fy;
        camera_matrix.at<double>(0, 2) = cx;
        camera_matrix.at<double>(1, 2) = cy;
    }

    void CameraCalibration::setDistortionCoefficients(double k1, double k2, double p1, double p2)
    {
        distortion_coeffs = cv::Mat(4, 1, cv::DataType<double>::type);
        distortion_coeffs.at<double>(0) = k1;
        distortion_coeffs.at<double>(1) = k2;
        distortion_coeffs.at<double>(2) = p1;
        distortion_coeffs.at<double>(3) = p2;
    }

    void CameraCalibration::setDistortionCoefficients(double k1, double k2, double p1, double p2, double k3)
    {
        distortion_coeffs = cv::Mat(5, 1, cv::DataType<double>::type);
        distortion_coeffs.at<double>(0) = k1;
        distortion_coeffs.at<double>(1) = k2;
        distortion_coeffs.at<double>(2) = p1;
        distortion_coeffs.at<double>(3) = p2;
        distortion_coeffs.at<double>(4) = k3;
    }

    void CameraCalibration::setDistortionCoefficients(double k1, double k2, double p1, double p2, double k3, double k4, double k5, double k6)
    {
        distortion_coeffs = cv::Mat(8, 1, cv::DataType<double>::type);
        distortion_coeffs.at<double>(0) = k1;
        distortion_coeffs.at<double>(1) = k2;
        distortion_coeffs.at<double>(2) = p1;
        distortion_coeffs.at<double>(3) = p2;
        distortion_coeffs.at<double>(4) = k3;
        distortion_coeffs.at<double>(5) = k4;
        distortion_coeffs.at<double>(6) = k5;
        distortion_coeffs.at<double>(7) = k6;
    }

    MapKeyFrame::MapKeyFrame() :
        frameId_(-1), optimized_(false), previousKeyframeId_(-1)
    {

    }

    const std::vector<cv::KeyPoint>& MapKeyFrame::keypoints(int cameraIdx)
    {
        return keypoints_[cameraIdx];
    }

    const cv::Mat& MapKeyFrame::descriptors(int cameraIdx)
    {
        return descriptors_[cameraIdx];
    }

    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> & MapKeyFrame::homogeneousKeypoints3d(int cameraIdx)
    {

        return keypoints3dh_C[cameraIdx];
    }

    void MapKeyFrame::descriptorsAsVec(int cameraIdx, std::vector<cv::Mat>& out)
    {
        out.clear();
        out.reserve(descriptors_[cameraIdx].rows);
        for (int i = 0; i < descriptors_[cameraIdx].rows; i++) {
            out.emplace_back(descriptors_[cameraIdx].row(i));
        }
    }

    void MapKeyFrame::setOptimizedTransform(const Eigen::Matrix4d& T_WS_in)
    {
        T_WS_Optimized_ = T_WS_in;
        optimized_ = true;
    }

    Eigen::Matrix4d MapKeyFrame::T_WS()
    {
        if (optimized_) {
            return T_WS_Optimized_;
        }
        else
            return T_WS_;
    }

    Eigen::Matrix4d MapKeyFrame::T_WC(int index)
    {
        if (index >= 0 && index < T_SC_.size()) {
            return T_WS()*T_SC_[index];
        }
        else
            return T_WS();
    }

    bool MultiCameraFrame::getImageByType(FrameType type, cv::Mat& out, int num/*=0 */)
    {
        int found = 0;
        for (size_t i = 0; i < images_.size(); i++) {
            if (image_types_[i] == type) {
                if (found == num) {
                    out = images_[i];
                    return true;
                }
                else {
                    found++;
                }
            }
        }
        return false;
    }

    bool MultiCameraFrame::getImage(cv::Mat& out, int num)
    {
        if (num < images_.size()) {
            out = images_[num];
            return true;
        }
        return false;
    }

    MultiCameraFrame::MultiCameraFrame(int frame_id /*= -1*/)
    {
        frameId_ = frame_id;
    }

    MultiCameraFrame::MultiCameraFrame(std::vector<cv::Mat> images, const Eigen::Matrix4d& T_KS, int frame_id /*= -1*/, int keyframe_id /*= -1*/)
    {
        this->images_ = images;
        this->T_KS_ = T_KS;
        frameId_ = frame_id;
        keyframeId_ = keyframe_id;
    }

    MultiCameraFrame::MultiCameraFrame(cv::Mat image, const Eigen::Matrix4d& T_KS, int frame_id /*= -1*/, int keyframe_id /*= -1*/)
    {
        images_.push_back(image);
        this->T_KS_ = T_KS;
        frameId_ = frame_id;
        keyframeId_ = keyframe_id;
    }

    Eigen::Matrix4d MultiCameraFrame::T_WS()
    {
        if (keyframeId_ != -1 && keyframe_.get() != nullptr)
            return keyframe_->T_WS()*T_KS_;
        else
            return T_KS_;
    }

    Eigen::Matrix4d MultiCameraFrame::T_WC(int index)
    {
        if (index >= 0 && index < T_SC_.size()) {
            return T_WS()*T_SC_[index];
        }
        else
            return T_WS();
    }

    void MultiCameraFrame::saveSimple(std::string dir)
    {
        std::stringstream filename;
        for (size_t i = 0; i < images_.size(); i++) {
            filename << dir << frameId_ << "_" << i << ".png";
            cv::imwrite(filename.str(), images_[i]);
        }

        //Add timesync data to timesync file
        std::ofstream kf_T;
        kf_T.open(dir + std::to_string(frameId_) + "_T.csv");
        Eigen::Matrix4d T = T_WS();
        kf_T << T(0, 0) << "," << T(0, 1) << "," << T(0, 2) << "," << T(0, 3)
            << "," << T(1, 0) << "," << T(1, 1) << "," << T(1, 2) << "," << T(1, 3)
            << "," << T(2, 0) << "," << T(2, 1) << "," << T(2, 2) << "," << T(2, 3)
            << "," << T(3, 0) << "," << T(3, 1) << "," << T(3, 2) << "," << T(3, 3);
        kf_T.close();
    }

}