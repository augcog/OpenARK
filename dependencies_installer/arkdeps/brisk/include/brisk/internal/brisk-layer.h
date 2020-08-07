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

#ifndef INTERNAL_BRISK_LAYER_H_
#define INTERNAL_BRISK_LAYER_H_

#include <memory>
#include <vector>

#include <agast/agast5-8.h>
#include <agast/oast9-16.h>

#include <brisk/internal/macros.h>

namespace brisk {
// If a per pixel threshold map with locally adaptive thresholds should be used
// instead of a global threshold for the entire image.
// This improves the repeatability of keypoints in low texture areas and leads
// to a more uniform keypoint distribution, however incurs a computational cost.
constexpr bool kUseThresholdMap = false;

// A layer in the Brisk detector pyramid.
class  BriskLayer {
 public:
  // Constructor arguments.
  struct  CommonParams {
    static const int HALFSAMPLE = 0;
    static const int TWOTHIRDSAMPLE = 1;
  };
  // Construct a base layer.
  BriskLayer(const cv::Mat& img, unsigned char upper_threshold, unsigned char lower_threshold,
             float scale = 1.0f, float offset = 0.0f);
  // Derive a layer.
  BriskLayer(const BriskLayer& layer, int mode, unsigned char upper_threshold,
             unsigned char lower_threshold);

  // Fast/Agast without non-max suppression.
  void GetAgastPoints(uint8_t threshold,
                      std::vector<cv::KeyPoint>* keypoints) const;

  // Get scores - this is in layer coordinates, not scale=1 coordinates!
  uint8_t GetAgastScore(int x, int y, uint8_t threshold) const;
  uint8_t GetAgastScore_5_8(int x, int y, uint8_t threshold) const;
  uint8_t GetAgastScore(float xf, float yf, uint8_t threshold,
                        float scale = 1.0f) const;

  // Accessors.
  inline const cv::Mat& img() const {
    return img_;
  }
  inline const cv::Mat& scores() const {
    return scores_;
  }
  inline float scale() const {
    return scale_;
  }
  inline float offset() const {
    return offset_;
  }
  int cols() const {
    return img_.cols;
  }
  int rows() const {
    return img_.rows;
  }

 private:
  // Access gray values (smoothed/interpolated).
  uint8_t Value(const cv::Mat& mat, float xf, float yf, float scale) const;
  // Calculate threshold map.
  void CalculateThresholdMap();

  // The image.
  cv::Mat img_;
  // Its Fast scores.
  cv::Mat scores_;
  // Its threshold map.
  cv::Mat thrmap_;
  // Coordinate transformation.
  float scale_;
  float offset_;
  // Agast detectors.
  std::shared_ptr<agast::OastDetector9_16> oastDetector_;
  std::shared_ptr<agast::AgastDetector5_8> agastDetector_5_8_;

  unsigned char upperThreshold_;
  unsigned char lowerThreshold_;
};
}  // namespace brisk
#endif  // INTERNAL_BRISK_LAYER_H_
