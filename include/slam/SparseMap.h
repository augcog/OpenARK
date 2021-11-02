#pragma once

#include "util/Types.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/eigen.hpp>
#include <DBoW2.h>
#include <DLoopDetector.h>
#include "PoseGraphSolver.h"
#include "CorrespondenceRansac.h"
#include "util/Util.h"
#include "PointCostSolver.h"

namespace ark{

/**
 * @brief This class 
 */
template<class TDescriptor, class F>
class SparseMap {
 public:

  SparseMap();

  void getFrames(std::vector<int>& frame_ids);

  void getTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut);

  void getMappedTrajectory(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut);

  int getNumKeyframes();

  MapKeyFrame::Ptr getCurrentKeyframe();

  MapKeyFrame::Ptr getKeyframe(int frameId);
  
  bool addKeyframe(MapKeyFrame::Ptr kf, MapKeyFrame::Ptr &loop_kf, Eigen::Affine3d &transformEstimate);

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
