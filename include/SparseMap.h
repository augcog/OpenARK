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
  useLoopClosures(false),matcher_(nullptr),bowId(0),currentKeyframeId(-1),lastKfTimestamp_(0)
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

  MapKeyFrame::Ptr getCurrentKeyframe(){
    return getKeyframe(currentKeyframeId);
  };
  MapKeyFrame::Ptr getKeyframe(int frameId){
    std::map<int,MapKeyFrame::Ptr>::iterator it = frameMap_.find(frameId);
    if(it!=frameMap_.end()){
      return it->second;
    }else {
      return MapKeyFrame::Ptr(nullptr);
    }
  }
  bool detectLoopClosure(MapKeyFrame::Ptr kf)
  {
     if(useLoopClosures && kf->timestamp_-lastKfTimestamp_>0.2*1e9)
     {
        lastKfTimestamp_=kf->timestamp_;
        //convert to descriptors to DBoW descriptor format
        std::vector<cv::Mat> bowDesc;
        kf->descriptorsAsVec(0,bowDesc);
        DLoopDetector::DetectionResult result;
        DBoW2::BowVector bowVec;
        DBoW2::FeatureVector featvec;
        detector_->detectLoop(kf->keypoints(0),bowDesc,result);
        MapKeyFrame::Ptr loop_kf;
        if(result.detection())
        {
          // std::cout << result.match << " " << bowId << std::endl; 
          loop_kf = bowFrameMap_[result.match];
          if (loop_kf) {
            cout << "detectLoopClosure: Loop found with image " << loop_kf->frameId_ << "!" << endl;
          } else {
            cout << "detectLoopClosure: Loop found and result is null\n";
          }
          return true;
        }
        else
        {
          //We only want to record a frame if it is not matched with another image
          //no need to duplicate
          return false; //pose added to graph, no loop detected, nothing left to do
        }
      }  
  }
  bool addKeyframe(MapKeyFrame::Ptr kf){
    //std::cout << "PROCESS KEYFRAME: " << kf->frameId_ << std::endl;
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

    std::vector<Eigen::Matrix4d> traj;
    getTrajectory(traj);
    double distanceTravelled = 0;
    // check the last 10 positions
    for(int i=traj.size()-1; i>0 && (traj.size() - i <= 10); i--){
        const auto &currentTranslation = traj[i].block<3,1>(0,3);
        const auto &previousTranslation = traj[i-1].block<3,1>(0,3);
        distanceTravelled += (currentTranslation-previousTranslation).norm();
    }
    bool shouldDetectLoopClosure = distanceTravelled > 0.5 && kf->timestamp_-lastKfTimestamp_>0.2*1e9;

    //only use one timestamp per second for loop closure
    if(useLoopClosures && shouldDetectLoopClosure){
      lastKfTimestamp_=kf->timestamp_;
      //convert to descriptors to DBoW descriptor format
      std::vector<cv::Mat> bowDesc;
      kf->descriptorsAsVec(0,bowDesc);

      DLoopDetector::DetectionResult result;
      DBoW2::BowVector bowVec;
      DBoW2::FeatureVector featvec;
      detector_->detectLoop(kf->keypoints(0),bowDesc,result);
      MapKeyFrame::Ptr loop_kf;
      if(result.detection())
      {
        std::cout << result.match << " " << bowId << std::endl; 
        loop_kf = bowFrameMap_[result.match];
        if (loop_kf) {
          cout << "addKeyframe: Loop found with image " << loop_kf->frameId_ << "!" << endl;
        } else {
          cout << "addKeyframe: Loop found and result is null\n";
        }
      }else{
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
      std::cout << "sizes: kf: " << kf->descriptors(0).rows << " loop_kf: "
        << loop_kf->descriptors(0).rows << " matches: " << matches.size() << std::endl;

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
      if(((float)numInliers)/correspondences.size()<0.3)
        return false; //transform unreliable
      std::cout << "inliers: " << numInliers << " / " << correspondences.size() << std::endl;

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

  DBoW2::EntryId lastEntry_;
  bool useLoopClosures;
  int currentKeyframeId;
  int bowId;

  SimplePoseGraphSolver graph_;
  double lastKfTimestamp_;



private: 

 };//class SparseMap


} //namespace ark

