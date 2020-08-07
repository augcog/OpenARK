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

#ifndef INTERNAL_SCORE_CALCULATOR_H_
#define INTERNAL_SCORE_CALCULATOR_H_

#ifdef __ARM_NEON__
#include <arm_neon.h>
#else
#include <emmintrin.h>
#include <tmmintrin.h>
#endif  // __ARM_NEON__
#include <vector>
#include <agast/wrap-opencv.h>

#include <brisk/internal/macros.h>

namespace brisk {

// Abstract base class to provide an interface for score calculation of any
// sort.
template<typename SCORE_TYPE>
class ScoreCalculator {
 public:
  typedef SCORE_TYPE Score_t;

  // Helper struct for point storage.
#ifdef USE_SIMPLE_POINT_WITH_SCORE
  struct PointWithScore {
    PointWithScore()
        : score(0),
          x(0),
          y(0) {
    }
    PointWithScore(Score_t score_, uint16_t x_, uint16_t y_)
        : score(score_),
          x(x_),
          y(y_) {
    }
    Score_t score;
    uint16_t x, y;
    // This is so terrible. but so fast:
    // TODO(slynen) Fix this: The operator says smaller than, but returns
    // larger than.
    bool operator<(const PointWithScore& other) const {
      return score > other.score;
    }
  };
#else
#error
  struct PointWithScore {
    PointWithScore():
    pt(cv::Point2i(0, 0)), Score(0) {}
    PointWithScore(cv::Point2i pt_, Score_t score_):
    pt(pt_), Score(score_) {}
    cv::Point2i pt;
    Score_t Score;
    inline bool operator<(const PointWithScore& other) const {
      return Score >= other.Score;
    }
  };
#endif

  // Constructor.
  ScoreCalculator() {
  }
  // Destructor.
  virtual ~ScoreCalculator() {
  }

  // Set image.
  void SetImage(const cv::Mat& img, bool initScores = true) {
    _img = img;
    if (initScores)
      InitializeScores();
  }

  // Calculate/get score - implement floating point and integer access.
  virtual inline double Score(double u, double v) const = 0;
  virtual inline Score_t Score(int u, int v) const = 0;

  // 2d maximum query.
  virtual void Get2dMaxima(std::vector<PointWithScore>& points,  // NOLINT
                           Score_t absoluteThreshold = 0) const = 0;

 protected:
  cv::Mat _img;  // The image we operate on.
  cv::Mat _scores;  // Store calculated scores.
  virtual void InitializeScores() = 0;
};
}  // namespace brisk
#endif  // INTERNAL_SCORE_CALCULATOR_H_
