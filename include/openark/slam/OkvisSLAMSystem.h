#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <brisk/brisk.h>
#include <opencv2/core/eigen.hpp>
#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include "openark/slam/SLAMSystem.h"
#include "openark/slam/SparseMap.h"
#include "openark/slam/SingleConsumerPriorityQueue.h"
#include "DBoW2.h"

namespace ark {

    typedef DBoW2::FORB::TDescriptor ORBDescType;
    typedef DBoW2::FORB ORBDesc;

    typedef DBoW2::FBRISK::TDescriptor BRISKDescType;
    typedef DBoW2::FORB BRISKDesc;

    /** Okvis-based SLAM system */
    template<class DescType, class Feat> class OkvisSLAMSystem : public SLAMSystem {

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

        void PushFrame(const MultiCameraFrame::Ptr frame);

        void PushIMU(const std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>>& imu);

        void PushIMU(const ImuPair& imu);

        void PushIMU(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d gyro);

        void Start();

        void RequestStop();

        void ShutDown();

        bool IsRunning();

        //underlying okvis estimator is reset due to tracking loss
        bool TrackingIsReset();

        void display();

        void getActiveFrames(std::vector<int>& frame_ids);

        void getTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut);

        void getTrajectoryWithFrameIds(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut);
        
        int getActiveMapIndex();

        bool getMapTrajectory(int index, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut);

        ~OkvisSLAMSystem();

    protected:
        void FrameConsumerLoop();

        void createNewMap();

        void setEnableLoopClosure(bool enableUseLoopClosures, std::string vocabPath, cv::DescriptorMatcher* matcher);

        bool detectLoopClosure(MapKeyFrame::Ptr kf, MapKeyFrame::Ptr &loop_kf,
                Eigen::Affine3d &transformEstimate);

        std::shared_ptr<SparseMap<DescType, Feat>> mergeMaps(
                std::shared_ptr<SparseMap<DescType, Feat>> olderMap,
                std::shared_ptr<SparseMap<DescType, Feat>> currentMap,
                MapKeyFrame::Ptr kf, MapKeyFrame::Ptr loop_kf, Eigen::Affine3d &transformEstimate);

        
        std::shared_ptr<SparseMap<DescType, Feat>> getActiveMap();
        
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
        std::map<int, std::shared_ptr<SparseMap<DescType, Feat>>> sparse_maps_;

        bool new_map_checker;
        int map_timer;
        int active_map_index;
        int map_id_counter_;
        std::string strVocFile;

        bool useLoopClosures_;
        std::shared_ptr<DLoopDetector::TemplatedLoopDetector<DescType, Feat>> detector_;
        std::shared_ptr<DBoW2::TemplatedVocabulary<DescType, Feat>> vocab_;
        std::shared_ptr<cv::DescriptorMatcher> matcher_;
        std::map<int, MapKeyFrame::Ptr> bowFrameMap_;
        int bowId_;
        double lastLoopClosureTimestamp_;
        // correction for convert an obj coordinate in other's map 
        // because reset okvis estimator also reset coordinate system
        Eigen::Matrix4d correction_{Eigen::Matrix4d::Identity()};

        static const int kMapCreationCooldownFrames_ = 15;
        static const int kMinimumKeyframes_ = 20;

        std::shared_ptr<okvis::ThreadedKFVio> okvis_estimator_;

    }; // OkvisSLAMSystem

    typedef OkvisSLAMSystem<ORBDescType, ORBDesc> OkvisSLAMSystemORBFeatures;
    typedef OkvisSLAMSystem<BRISKDescType, BRISKDesc> OkvisSLAMSystemBRISKFeatures;

}//ark
