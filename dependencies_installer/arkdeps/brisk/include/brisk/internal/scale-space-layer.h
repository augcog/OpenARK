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

#ifndef INTERNAL_SCALE_SPACE_LAYER_H_
#define INTERNAL_SCALE_SPACE_LAYER_H_

#include <vector>


#include <brisk/internal/macros.h>

namespace brisk {

// A generic layer to be used within the ScaleSpace class.
template<class SCORE_CALCULATOR_T>
class ScaleSpaceLayer {
 public:
  typedef SCORE_CALCULATOR_T ScoreCalculator_t;
  ScaleSpaceLayer() { }
  ScaleSpaceLayer(const cv::Mat& img, bool initScores = true);  // Octave 0.
  ScaleSpaceLayer(ScaleSpaceLayer<ScoreCalculator_t>* layerBelow,
                  bool initScores = true);  // For successive construction.

  void Create(const cv::Mat& img, bool initScores = true);  // Octave 0.
  void Create(ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores =
                  true);  // For successive construction.

  void SetUniformityRadius(double radius);
  void SetMaxNumKpt(size_t maxNumKpt) {
    _maxNumKpt = maxNumKpt;
  }
  void SetAbsoluteThreshold(double absoluteThreshold) {
    _absoluteThreshold = absoluteThreshold;
  }

  // Feature detection.
  void DetectScaleSpaceMaxima(std::vector<cv::KeyPoint>& keypoints,  // NOLINT
                              bool enforceUniformity = true, bool doRefinement =
                                  true,
                              bool usePassedKeypoints = false);

  // Subsampling.
  // Half sampling.
  static inline bool Halfsample(const cv::Mat& srcimg, cv::Mat& dstimg);
  // Two third sampling.
  static inline bool Twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg);

 protected:
  // Utilities.
  inline double ScoreAbove(double u, double v);
  inline double ScoreBelow(double u, double v);

  // 1d (scale) refinement.
  __inline__ float Refine1D(const float s_05, const float s0, const float s05,
                            float& max);  // Around octave.
  __inline__ float Refine1D_1(const float s_05, const float s0, const float s05,
                              float& max);  // Around intra.

  // 2D maximum refinement:
  __inline__ float Subpixel2D(const double s_0_0, const double s_0_1,
                              const double s_0_2, const double s_1_0,
                              const double s_1_1, const double s_1_2,
                              const double s_2_0, const double s_2_1,
                              const double s_2_2, float& delta_x,
                              float& delta_y);

  // Layer properties.
  bool _isOctave;
  int _layerNumber;

  // Have a reference to the image for convenience:
  cv::Mat _img;

  // The score calculation.
  ScoreCalculator_t _scoreCalculator;

  // Remember next and previous layer.
  ScaleSpaceLayer* _aboveLayer_ptr;
  ScaleSpaceLayer* _belowLayer_ptr;

  // For coordinate transformations:
  double _offset_above, _offset_below;
  double _scale_above, _scale_below;
  double _scale;
  double _offset;

  // Uniformity enforcement related.
  double _radius;
  size_t _maxNumKpt;
  double _absoluteThreshold;
  cv::Mat _LUT;
};
}  // namespace brisk

#include "./scale-space-layer-inl.h"

#endif  // INTERNAL_SCALE_SPACE_LAYER_H_
