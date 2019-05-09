#pragma once

#include "SLAMSystem.h"
#include "SparseMap.h"
#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <thread>
#include <opencv2/core/eigen.hpp>
#include "SingleConsumerPriorityQueue.h"
#include <atomic>
#include <brisk/brisk.h>

namespace ark {
    /** Okvis-based SLAM system */
    class OkvisSLAMSystem : public SLAMSystem {

        struct WrappedMultiCameraFrame {
            MultiCameraFrame::Ptr frame;
            bool operator<(const WrappedMultiCameraFrame& right) const
            {
                return frame->timestamp_ > right.frame->timestamp_;
            }
        };

        struct StampedFrameData {
            okvis::OutFrameData::Ptr data;
            okvis::Time timestamp;
            bool operator<(const StampedFrameData& right) const
            {
                return timestamp > right.timestamp;
            }
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OkvisSLAMSystem(const std::string &strVocFile, const std::string &strSettingsFile);

        //void PushFrame(const std::vector<cv::Mat>& images, const double &timestamp);

        //void PushFrame(const cv::Mat image, const double &timestamp);

        void PushFrame(const MultiCameraFrame::Ptr frame);

        void PushIMU(const std::vector<ImuPair>& imu);

        void PushIMU(const ImuPair& imu);

        void PushIMU(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d gyro);

        void Start();

        void RequestStop();

        void ShutDown();

        bool IsRunning();

        void display();

        void getTrajectory(std::vector<Eigen::Matrix4d>& trajOut);

        ~OkvisSLAMSystem();

        
        std::shared_ptr<okvis::ThreadedKFVio> okvis_estimator_;

    protected:
        void KeyFrameConsumerLoop();

        void FrameConsumerLoop();

    private:
        okvis::Time start_;
        okvis::Time t_imu_;
        okvis::Duration deltaT_;
        okvis::VioParameters parameters_;
        SingleConsumerPriorityQueue<WrappedMultiCameraFrame> frame_queue_;
        SingleConsumerPriorityQueue<StampedFrameData> frame_data_queue_;
        std::thread frameConsumerThread_;
        int num_frames_;
        std::atomic<bool> kill;
        SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK> sparseMap_;

    }; // OkvisSLAMSystem

}//ark
