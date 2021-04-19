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
#include <vector>
#include <memory>

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
        OkvisSLAMSystem(const std::string &strVocFile, const std::string &strSettingsFile);

        OkvisSLAMSystem(const std::string &strVocFile, okvis::VioParameters& parameters);

        //void PushFrame(const std::vector<cv::Mat>& images, const double &timestamp);

        //void PushFrame(const cv::Mat image, const double &timestamp);

        void PushFrame(const MultiCameraFrame::Ptr frame);

        void PushIMU(const std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>>& imu);

        void PushIMU(const ImuPair& imu);

        void PushIMU(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);

        void Start();

        void RequestStop();

        void ShutDown();

        bool IsRunning();

        void display();

        void getActiveFrames(std::vector<int>& frame_ids);

        void getTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut); // Moon: Cause 4 = Cause 2.a + Cause 3.

		void getMappedTrajectory(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut); // Moon: Cause 4 = Cause 2.a + Cause 3.
        
        ~OkvisSLAMSystem();

        std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>> getActiveMap();


        int getActiveMapIndex() {
            return active_map_index;
        }
      
        std::shared_ptr<okvis::ThreadedKFVio> okvis_estimator_;

        std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>> getMap(int index) {
            if (sparse_maps_.find(index) == sparse_maps_.end()) {
                return nullptr;
            } else {
                return sparse_maps_[index];
            }
        }

    protected:
        void KeyFrameConsumerLoop();

        void FrameConsumerLoop();

        void createNewMap();

        void setEnableLoopClosure(bool enableUseLoopClosures, std::string vocabPath,
                bool binaryVocab, cv::DescriptorMatcher* matcher);

        bool detectLoopClosure(MapKeyFrame::Ptr kf, MapKeyFrame::Ptr &loop_kf,
                Eigen::Affine3d &transformEstimate);

        std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>> mergeMaps(
                std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>> olderMap,
                std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>> currentMap,
                MapKeyFrame::Ptr kf, MapKeyFrame::Ptr loop_kf, Eigen::Affine3d &transformEstimate);
        
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
        std::map<int, std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>>> sparse_maps_;
        bool new_map_checker;
        int map_timer;
        int active_map_index;
        int map_id_counter_;
        std::string strVocFile;

        bool useLoopClosures_;
        std::shared_ptr<DLoopDetector::TemplatedLoopDetector<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK> >detector_;
        std::shared_ptr<DBoW2::TemplatedVocabulary<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK> >vocab_;
        std::shared_ptr<cv::DescriptorMatcher> matcher_;
        std::map<int, MapKeyFrame::Ptr> bowFrameMap_;
        int bowId_;
        double lastLoopClosureTimestamp_;
        // correction for convert an obj coordinate in other's map 
        // because reset okvis estimator also reset coordinate system
        Eigen::Matrix4d correction_{Eigen::Matrix4d::Identity()};

        static const int kMapCreationCooldownFrames_ = 15;
        static const int kMinimumKeyframes_ = 20;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }; // OkvisSLAMSystem

}//ark
