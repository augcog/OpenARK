#pragma once
#include "openark/util/Types.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/eigen.hpp>
#include <DBoW2.h>
#include <DLoopDetector.h>
#include "openark/slam/PoseGraphSolver.h"
#include "openark/slam/CorrespondenceRansac.h"
#include "openark/util/Util.h"
#include "openark/slam/PointCostSolver.h"

namespace ark{

/**
 * @brief This class 
 */
template<class TDescriptor, class F>
class SparseMap {
    public:

    SparseMap():
    currentKeyframeId(-1)
    {

    }

    void getFrames(std::vector<int>& frame_ids){
        for (auto frame : frameMap_) {
            frame_ids.push_back(frame.first);
        }
    }

    void getTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut){
        trajOut.resize(frameMap_.size());
        size_t i=0;
        for(std::map<int, MapKeyFrame::Ptr>::iterator frame = frameMap_.begin(); 
            frame!=frameMap_.end(); frame++, i++){
            trajOut[i] = frame->second->T_WS();
        }
    }

    void getTrajectoryWithFrameIds(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut){ 
        for(std::map<int, MapKeyFrame::Ptr>::iterator frame = frameMap_.begin(); 
            frame!=frameMap_.end(); frame++){
            frameIdOut.push_back(frame->first);
            trajOut.push_back(frame->second->T_WC(3));
        }
    }

    int getNumKeyframes() {
        return frameMap_.size();
    }

    MapKeyFrame::Ptr getCurrentKeyframe(){
        return getKeyframe(currentKeyframeId);
    };

    MapKeyFrame::Ptr getKeyframe(int frameId){
        std::map<int,MapKeyFrame::Ptr>::iterator it = frameMap_.find(frameId);
        if(it!=frameMap_.end()){
            return it->second;
        }else{
            return MapKeyFrame::Ptr(nullptr);
        }
    }
  
    bool addKeyframe(MapKeyFrame::Ptr kf, MapKeyFrame::Ptr &loop_kf, Eigen::Affine3d &transformEstimate) {
    //std::cout << "PROCESS KEYFRAME: " << kf->frameId_ << std::endl;

    frameMap_[kf->frameId_]=kf;
    kf->previousKeyframeId_ = currentKeyframeId;
    kf->previousKeyframe_ = getCurrentKeyframe();

    Eigen::Matrix4d T_K1K2; 
    if (kf->previousKeyframe_.get()!=nullptr){
        T_K1K2 = kf->previousKeyframe_->T_WS_.inverse()*kf->T_WS_;
        graph_.AddConstraint(kf->previousKeyframeId_,kf->frameId_,T_K1K2);
        kf->setOptimizedTransform(kf->previousKeyframe_->T_WS()*T_K1K2);
    } else 
        T_K1K2 = Eigen::Matrix4d::Identity();

    currentKeyframeId = kf->frameId_;
    graph_.AddPose(kf->frameId_,kf->T_WS());

    if (loop_kf != nullptr) {
        //TODO: Try refining pose estimate in image space, image space is really the proper way to do this
        //TODO: Reduced pose graph and ISAM methods will allow a longer runtime
        graph_.AddConstraint(kf->frameId_,loop_kf->frameId_, kf->T_SC_[2]*transformEstimate.inverse().matrix()*kf->T_SC_[2].inverse());
        graph_.optimize();
        for(std::map<int, MapKeyFrame::Ptr>::iterator frame=frameMap_.begin();
        frame!=frameMap_.end(); frame++){
        frame->second->setOptimizedTransform(graph_.getTransformById(frame->first));
        }
        return true;
    }
        return false;
    }

    std::map<int, MapKeyFrame::Ptr> frameMap_;

    DBoW2::EntryId lastEntry_;
    int currentKeyframeId;

    SimplePoseGraphSolver graph_;
    static constexpr double LOOP_CLOSURE_DISTANCE_THRESHOLD = 0.0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private: 

    };//class SparseMap

} //namespace ark