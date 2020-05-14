#pragma once

#include "Types.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/eigen.hpp>
#include <DBoW2.h>
#include <DLoopDetector.h>
#include "PoseGraphSolver.h"
#include "CorrespondenceRansac.h"
#include "Util.h"
#include "PointCostSolver.h"

namespace ark{

/**
 * @brief This class 
 */
template<class TDescriptor, class F>
class SparseMap {
 public:

  SparseMap():
  useLoopClosures(false),matcher_(nullptr),bowId(0),currentKeyframeId(-1),lastKfTimestamp_(0), lastKfTimestampDetect_(0)
  {

  }

  void setEnableLoopClosure(bool enableUseLoopClosures = false, std::string vocabPath = "", bool binaryVocab = true, cv::DescriptorMatcher* matcher = nullptr){
      matcher_.reset(matcher);
      useLoopClosures = enableUseLoopClosures;
      if(useLoopClosures){
        std::cout << "Loading Vocabulary From: " << vocabPath << std::endl;
        vocab_.reset(new DBoW2::TemplatedVocabulary<TDescriptor, F>());
        if(!binaryVocab){
          vocab_->load(vocabPath);
        }else{
          vocab_->binaryLoad(vocabPath); //Note: Binary Loading only supported for ORB/BRISK vocabularies
        }
        //vocab_->save(vocabPath+std::string(".tst"));
        std::cout << "Vocabulary Size: " << vocab_->size() << std::endl;
        typename DLoopDetector::TemplatedLoopDetector<TDescriptor, F>::Parameters detectorParams(0,0);
        //can change specific parameters if we want
        // Parameters given by default are:
        // use nss = true
        // alpha = 0.3
        // k = 3
        // geom checking = GEOM_DI
        detectorParams.k=2;
        detectorParams.di_levels = 4;
        detector_.reset(new DLoopDetector::TemplatedLoopDetector<TDescriptor, F>(*vocab_,detectorParams));
        std::cout << "Map Initialized\n";
      }
  }

  void getTrajectory(std::vector<Eigen::Matrix4d>& trajOut){
    trajOut.resize(frameMap_.size());
    size_t i=0;
    for(std::map<int, MapKeyFrame::Ptr>::iterator frame = frameMap_.begin(); 
        frame!=frameMap_.end(); frame++, i++){
      trajOut[i] = frame->second->T_WS();
    }
  }

  void getMappedTrajectory(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d>& trajOut){
  frameIdOut.resize(frameMap_.size());
    trajOut.resize(frameMap_.size());
    size_t i=0;
    for(std::map<int, MapKeyFrame::Ptr>::iterator frame = frameMap_.begin(); 
        frame!=frameMap_.end(); frame++, i++){
      frameIdOut[i] = frame->first;
      trajOut[i] = frame->second->T_WC(3);
    }
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
  bool detectLoopClosure(MapKeyFrame::Ptr kf)
  {
    bool shouldDetectLoopClosure = kf->timestamp_-lastKfTimestampDetect_>0.2*1e9;
    if(useLoopClosures && shouldDetectLoopClosure) {
      std::vector<cv::Mat> bowDesc;
      kf->descriptorsAsVec(0,bowDesc);
      DLoopDetector::DetectionResult result;
      auto local_keypoints = kf->keypoints(0);
      detector_->detectLoop_query(local_keypoints,bowDesc,result);
      MapKeyFrame::Ptr loop_kf;
      if(result.detection()){
        loop_kf = bowFrameMap_[result.match];
      }else{
        return false; //pose added to graph, no loop detected, nothing left to do
      }
      std::vector<cv::DMatch> matches; 
      //query,train
      matcher_->match(kf->descriptors(0),loop_kf->descriptors(0), matches);
      std::cout << "detectLoopClosure: " << "sizes: kf: " << kf->descriptors(0).rows << " loop_kf: "
        << loop_kf->descriptors(0).rows << " matches: " << matches.size() << " loop_kf id: " << loop_kf->frameId_ << std::endl;

      //get feature point clouds
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i=0; i<kf->keypoints(0).size(); i++){
        Eigen::Vector4d kp3dh_C = kf->homogeneousKeypoints3d(0)[i];
        kf_feat_cloud->points.push_back(pcl::PointXYZ(kp3dh_C[0],kp3dh_C[1],kp3dh_C[2]));
      } 
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr loop_kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i=0; i<loop_kf->keypoints(0).size(); i++){
        Eigen::Vector4d kp3dh_C = loop_kf->homogeneousKeypoints3d(0)[i];
        loop_kf_feat_cloud->points.push_back(pcl::PointXYZ(kp3dh_C[0],kp3dh_C[1],kp3dh_C[2]));
      }           

      //convert DMatch to correspondence
      std::vector<int> correspondences(matches.size());
      for(int i=0; i<matches.size(); i++){
        if(loop_kf->homogeneousKeypoints3d(0)[matches[i].queryIdx][3]!=0 && loop_kf->homogeneousKeypoints3d(0)[matches[i].trainIdx][3]!=0)
          correspondences[matches[i].queryIdx]=matches[i].trainIdx;
        else
          correspondences[matches[i].queryIdx]=-1;;
      }

      int numInliers;
      std::vector<bool> inliers;
      Eigen::Affine3d transformEstimate;
      //find initial transform estimate using feature points
      CorrespondenceRansac<pcl::PointXYZ>::getInliersWithTransform(
            kf_feat_cloud, loop_kf_feat_cloud, correspondences,
            3, 0.2, 50, numInliers, inliers, transformEstimate);
      if(((float)numInliers)/correspondences.size()<0.3) {
        return false; 
      }
      // store the correction
      correction = loop_kf->T_WS_ * kf->T_WS_.inverse();
      return true;
      }
    return false;
  }
  bool addKeyframe(MapKeyFrame::Ptr kf){
    //std::cout << "PROCESS KEYFRAME: " << kf->frameId_ << std::endl;
    // apply correction
    kf->T_WS_ = correction * kf->T_WS_;
    frameMap_[kf->frameId_]=kf;
    kf->previousKeyframeId_ = currentKeyframeId;
    kf->previousKeyframe_ = getCurrentKeyframe();

    Eigen::Matrix4d T_K1K2; 
    if(kf->previousKeyframe_.get()!=nullptr){
      T_K1K2 = kf->previousKeyframe_->T_WS_.inverse()*kf->T_WS_;
      graph_.AddConstraint(kf->previousKeyframeId_,kf->frameId_,T_K1K2);
      kf->setOptimizedTransform(kf->previousKeyframe_->T_WS()*T_K1K2);
    }else 
      T_K1K2 = Eigen::Matrix4d::Identity();

    currentKeyframeId = kf->frameId_;
    graph_.AddPose(kf->frameId_,kf->T_WS());

    bool shouldDetectLoopClosure = kf->timestamp_-lastKfTimestamp_>0.2*1e9;

    //only use one timestamp per second for loop closure
    if(useLoopClosures && shouldDetectLoopClosure){
      lastKfTimestamp_=kf->timestamp_;
      //convert to descriptors to DBoW descriptor format
      std::vector<cv::Mat> bowDesc;
      kf->descriptorsAsVec(0,bowDesc);

      DLoopDetector::DetectionResult result;
      DLoopDetector::DetectionResult queryResult;
      DBoW2::BowVector bowVec;
      DBoW2::FeatureVector featvec;
      auto local_keypoints = kf->keypoints(0);
      detector_->detectLoop(kf->keypoints(0),bowDesc,result);
      MapKeyFrame::Ptr loop_kf;
      const auto addKfDetectResult = result.detection();

      if(addKfDetectResult){
        loop_kf = bowFrameMap_[result.match];
      } else {
        //We only want to record a frame if it is not matched with another image
        //no need to duplicate
        bowFrameMap_[bowId]=kf;
        bowId++;
        return false; //pose added to graph, no loop detected, nothing left to do
      }

      //transform estimation
      //TODO: should move to function to be set as one of a variety of methods

      //brute force matching
      std::vector<cv::DMatch> matches; 
      //query,train
      matcher_->match(kf->descriptors(0),loop_kf->descriptors(0), matches);
      std::cout << "addKeyframe: " << "sizes: kf: " << kf->descriptors(0).rows << " loop_kf: "
        << loop_kf->descriptors(0).rows << " matches: " << matches.size() << " loop_kf id: " << loop_kf->frameId_ << std::endl;

      //get feature point clouds
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i=0; i<kf->keypoints(0).size(); i++){
        Eigen::Vector4d kp3dh_C = kf->homogeneousKeypoints3d(0)[i];
        kf_feat_cloud->points.push_back(pcl::PointXYZ(kp3dh_C[0],kp3dh_C[1],kp3dh_C[2]));
      } 
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr loop_kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i=0; i<loop_kf->keypoints(0).size(); i++){
        Eigen::Vector4d kp3dh_C = loop_kf->homogeneousKeypoints3d(0)[i];
        loop_kf_feat_cloud->points.push_back(pcl::PointXYZ(kp3dh_C[0],kp3dh_C[1],kp3dh_C[2]));
      }           

      //convert DMatch to correspondence
      std::vector<int> correspondences(matches.size());
      for(int i=0; i<matches.size(); i++){
        if(loop_kf->homogeneousKeypoints3d(0)[matches[i].queryIdx][3]!=0 && loop_kf->homogeneousKeypoints3d(0)[matches[i].trainIdx][3]!=0)
          correspondences[matches[i].queryIdx]=matches[i].trainIdx;
        else
          correspondences[matches[i].queryIdx]=-1;;
      }

      int numInliers;
      std::vector<bool> inliers;
      Eigen::Affine3d transformEstimate;
      //find initial transform estimate using feature points
      CorrespondenceRansac<pcl::PointXYZ>::getInliersWithTransform(
            kf_feat_cloud, loop_kf_feat_cloud, correspondences,
            3, 0.2, 50, numInliers, inliers, transformEstimate);
      if(((float)numInliers)/correspondences.size()<0.3){
        return false; 
      }

      transformEstimate = PointCostSolver<pcl::PointXYZ>::solve(kf_feat_cloud,loop_kf_feat_cloud,
                                            correspondences, inliers, transformEstimate);

      std::cout << transformEstimate.matrix() << std::endl;
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


  std::unique_ptr<DBoW2::TemplatedVocabulary<TDescriptor, F> >vocab_;
  //std::unique_ptr<DBoW2::TemplatedDatabase<TDescriptor, F> > db_;
  std::unique_ptr<DLoopDetector::TemplatedLoopDetector<TDescriptor, F> >detector_;
  std::shared_ptr<cv::DescriptorMatcher> matcher_;
  std::map<int, MapKeyFrame::Ptr> frameMap_;
  std::map<int, MapKeyFrame::Ptr> bowFrameMap_;
  // correction for convert an obj coordinate in other's map 
  // because reset okvis estimator also reset coordinate system
  Eigen::Matrix4d correction{Eigen::Matrix4d::Identity()};

  DBoW2::EntryId lastEntry_;
  bool useLoopClosures;
  int currentKeyframeId;
  int bowId;

  SimplePoseGraphSolver graph_;
  double lastKfTimestamp_;
  double lastKfTimestampDetect_;
  static constexpr double LOOP_CLOSURE_DISTANCE_THRESHOLD = 0.0;


private: 

 };//class SparseMap


} //namespace ark

