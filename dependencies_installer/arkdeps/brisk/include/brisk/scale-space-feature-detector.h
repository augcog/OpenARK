/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 All rights reserved.

 This is the Author's implementation of BRISK: Binary Robust Invariant
 Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
 some of which are described in [2].

 [1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
     Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
     International Conference on Computer Vision (ICCV), 2011.

 [2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
     Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

 This file is part of BRISK.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BRISK_SCALE_SPACE_FEATURE_DETECTOR_H_
#define BRISK_SCALE_SPACE_FEATURE_DETECTOR_H_

#include <algorithm>
#include <limits>
#include <vector>
#include <stdexcept>
#include <agast/wrap-opencv.h>

#include <brisk/internal/macros.h>
#include <brisk/internal/scale-space-layer.h>

namespace brisk {

// Uses the common feature interface to construct a generic
// scale space detector from a given ScoreCalculator.
template<class SCORE_CALCULATOR_T>
class ScaleSpaceFeatureDetector : public cv::FeatureDetector
{
 public:
  ScaleSpaceFeatureDetector(double uniformityRadius, size_t octaves, 
                            double absoluteThreshold = 0, size_t maxNumKpt =
                                std::numeric_limits<size_t>::max())
      : _uniformityRadius(uniformityRadius),
        _octaves(octaves),
        _absoluteThreshold(absoluteThreshold),
        _maxNumKpt(maxNumKpt)
  {
    scaleSpaceLayers.resize(std::max(_octaves * 2, size_t(1)));
  }

  typedef SCORE_CALCULATOR_T ScoreCalculator_t;

#if CV_MAJOR_VERSION < 3
  void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
              const cv::Mat& mask = cv::Mat()) const {
#else
  void detect(cv::InputArray image_,
              std::vector<cv::KeyPoint>& keypoints,
              cv::InputArray mask_ = cv::noArray()) override {
  cv::Mat image = image_.getMat();
  cv::Mat mask = mask_.getMat(); 
#endif 
    if(!image.isContinuous() || image.channels()!=1 || image.elemSize()!=1) {
      throw std::runtime_error("BRISK requires continuous 1-channel 8-bit images");
    }
    if (image.empty()) {
      return;
    }
    if(!(mask.empty() ||
        (mask.type() == CV_8UC1 && mask.rows == image.rows && mask.cols == image.cols))){
      throw std::runtime_error("mask must be empty or CV_8UC1 type with image dimensions");
    }
    detectImpl(image, keypoints, mask);
  }

 protected:
#if CV_MAJOR_VERSION < 3
  void detectImpl(const cv::Mat& image,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat& mask = cv::Mat()) const override {
#else
  void detectImpl(const cv::Mat& image,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat& mask = cv::Mat()) const {
#endif

    if(!mask.empty()) {
       throw std::runtime_error("BRISK does not support masks yet");
       return;
    }
    // Find out, if we should use the provided keypoints.
    bool usePassedKeypoints = false;
    if (keypoints.size() > 0)
      usePassedKeypoints = true;
    else
      keypoints.reserve(4000);  // Possibly speeds up things.

    // find the factors for per-layer maximum enforcement
    double factor = 1.0;
    for (size_t i = 1; i < _octaves * 2; ++i) {
      if (i % 2 == 0) {
        factor += 1.0 / pow(2.0, i / 2);
      } else {
        factor += 1.0 / (1.5 * pow(2.0, i / 2));
      }
    }

    // Construct scale space layers.
    double maxNumKpt = _maxNumKpt / factor;
    scaleSpaceLayers[0].Create(image, !usePassedKeypoints);
    scaleSpaceLayers[0].SetUniformityRadius(_uniformityRadius);
    scaleSpaceLayers[0].SetMaxNumKpt(maxNumKpt);
    scaleSpaceLayers[0].SetAbsoluteThreshold(_absoluteThreshold);
    for (size_t i = 1; i < _octaves * 2; ++i) {
      if (i % 2 == 0) {
        maxNumKpt = _maxNumKpt / (factor * pow(2.0, i / 2));
      } else {
        maxNumKpt = _maxNumKpt / (factor * 1.5 * pow(2.0, i / 2));
      }
      scaleSpaceLayers[i].Create(&scaleSpaceLayers[i - 1], !usePassedKeypoints);
      scaleSpaceLayers[i].SetUniformityRadius(_uniformityRadius);
      scaleSpaceLayers[i].SetMaxNumKpt(maxNumKpt);
      scaleSpaceLayers[i].SetAbsoluteThreshold(_absoluteThreshold);
    }
    for (size_t i = 0; i < scaleSpaceLayers.size(); ++i) {
      // Only do refinement, if no keypoints are passed.
      scaleSpaceLayers[i].DetectScaleSpaceMaxima(keypoints, true,
                                                 !usePassedKeypoints,
                                                 usePassedKeypoints);
    }
  }

  double _uniformityRadius;
  size_t _octaves;
  double _absoluteThreshold;
  size_t _maxNumKpt;
  mutable std::vector<brisk::ScaleSpaceLayer<ScoreCalculator_t> > scaleSpaceLayers;
};
}  // namespace brisk

#endif  // BRISK_SCALE_SPACE_FEATURE_DETECTOR_H_
