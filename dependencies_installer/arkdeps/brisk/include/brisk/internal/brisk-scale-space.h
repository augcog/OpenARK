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

#ifndef INTERNAL_BRISK_SCALE_SPACE_H_
#define INTERNAL_BRISK_SCALE_SPACE_H_

#include <vector>


#include <brisk/internal/brisk-layer.h>
#include <brisk/internal/macros.h>

namespace brisk {
class  BriskScaleSpace {
 public:
  // Construct telling the octaves number:
  BriskScaleSpace(uint8_t octaves = 3, bool suppress_scale_nonmaxima = true);
  ~BriskScaleSpace();

  // Construct the image pyramids.
  void ConstructPyramid(const cv::Mat& image, unsigned char threshold,
                        unsigned char overwrite_lower_thres = kDefaultLowerThreshold);

  // Get Keypoints.
  void GetKeypoints(std::vector<cv::KeyPoint>* keypoints) const;

 protected:
  // Nonmax suppression:
  __inline__ bool IsMax2D(const uint8_t layer, const int x_layer,
                          const int y_layer) const;
  // 1D (scale axis) refinement:
  __inline__ float Refine1D(const float s_05, const float s0, const float s05,
                            float& max) const;  // Around octave.
  __inline__ float Refine1D_1(const float s_05, const float s0, const float s05,
                              float& max) const;  // Around intra.
  __inline__ float Refine1D_2(const float s_05, const float s0, const float s05,
                              float& max) const;  // Around octave 0 only.
  // 2D maximum refinement:
  __inline__ float Subpixel2D(const int s_0_0, const int s_0_1, const int s_0_2,
                              const int s_1_0, const int s_1_1, const int s_1_2,
                              const int s_2_0, const int s_2_1, const int s_2_2,
                              float& delta_x, float& delta_y) const;
  // 3D maximum refinement centered around (x_layer,y_layer).
  __inline__ float Refine3D(const uint8_t layer, const int x_layer,
                            const int y_layer, float& x, float& y, float& scale,
                            bool& ismax) const;

  // Interpolated score access with recalculation when needed:
  __inline__ int GetScoreAbove(const uint8_t layer, const int x_layer,
                               const int y_layer) const;
  __inline__ int GetScoreBelow(const uint8_t layer, const int x_layer,
                               const int y_layer) const;

  // Teturn the maximum of score patches above or below.
  __inline__ float GetScoreMaxAbove(const uint8_t layer, const int x_layer,
                                    const int y_layer, const int threshold,
                                    bool& ismax, float& dx, float& dy) const;
  __inline__ float GetScoreMaxBelow(const uint8_t layer, const int x_layer,
                                    const int y_layer, const int threshold,
                                    bool& ismax, float& dx, float& dy) const;

  // The image pyramids:
  uint8_t layers_;
  std::vector<brisk::BriskLayer> pyramid_;

  // Agast:
  uint8_t threshold_;

  // Some constant parameters:
  static const float kBasicSize_;

  // Thresholds for the scale determination.
  static const int kDropThreshold_;
  static const int kMaxThreshold_;
  static const int kMinDrop_;

  // Detection thresholds: upper and lower bounds.
  static const unsigned char kDefaultUpperThreshold;
  static const unsigned char kDefaultLowerThreshold;

  bool suppressScaleNonmaxima_;
};
}  // namespace brisk
#endif  // INTERNAL_BRISK_SCALE_SPACE_H_
