/*
 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AGAST_WRAP_OPENCV_H_
#define AGAST_WRAP_OPENCV_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma GCC diagnostic pop

namespace cv {

inline float& KeyPointX(cv::KeyPoint& keypoint) {  // NOLINT
  return keypoint.pt.x;
}
inline const float& KeyPointX(const cv::KeyPoint& keypoint) {
  return keypoint.pt.x;
}
inline float& KeyPointY(cv::KeyPoint& keypoint) {  // NOLINT
  return keypoint.pt.y;
}
inline const float& KeyPointY(const cv::KeyPoint& keypoint) {
  return keypoint.pt.y;
}
inline float& KeyPointAngle(cv::KeyPoint& keypoint) {  // NOLINT
  return keypoint.angle;
}
inline const float& KeyPointAngle(const cv::KeyPoint& keypoint) {
  return keypoint.angle;
}
inline float& KeyPointSize(cv::KeyPoint& keypoint) {  // NOLINT
  return keypoint.size;
}
inline const float& KeyPointSize(const cv::KeyPoint& keypoint) {
  return keypoint.size;
}
inline float& KeyPointResponse(cv::KeyPoint& keypoint) {  // NOLINT
  return keypoint.response;
}
inline const float& KeyPointResponse(const cv::KeyPoint& keypoint) {
  return keypoint.response;
}
inline int& KeyPointOctave(cv::KeyPoint& keypoint) {  // NOLINT
  return keypoint.octave;
}
inline const int& KeyPointOctave(const cv::KeyPoint& keypoint) {
  return keypoint.octave;
}
}  // namespace cv

#endif  // AGAST_WRAP_OPENCV_H_
