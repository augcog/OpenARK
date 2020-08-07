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

#ifndef INTERNAL_SCALE_SPACE_LAYER_INL_H_
#define INTERNAL_SCALE_SPACE_LAYER_INL_H_

#include <algorithm>
#include <vector>

#include <brisk/internal/uniformity-enforcement.h>
#include <brisk/internal/image-down-sampling.h>
//#include <brisk/internal/timer.h>

namespace brisk {
template<class SCORE_CALCULATOR_T>
ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScaleSpaceLayer(const cv::Mat& img,
                                                     bool /*initScores*/) {
  Create(img);
}

template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::Create(const cv::Mat& img,
                                                 bool initScores) {
  // Octave 0.
  _isOctave = true;
  _layerNumber = 0;

  // No adjacent layers (yet).
  _belowLayer_ptr = 0;
  _aboveLayer_ptr = 0;

  // Pass image and initialize score calculation.
  _scoreCalculator.SetImage(img, initScores);
  _img = img;

  // Scales and offsets.
  _offset_above = -0.25;
  _offset_below = 1.0 / 6.0;
  _scale_above = 2.0 / 3.0;
  _scale_below = 4.0 / 3.0;
  _scale = 1.0;
  _offset = 0.0;

  // By default no uniformity radius.
  _radius = 1;

  // Abs. threshold (for noise rejection).
  _absoluteThreshold = 0;

  // Generic mask.
  _LUT = cv::Mat::zeros(2 * 16 - 1, 2 * 16 - 1, CV_32F);
  for (int x = 0; x < 2 * 16 - 1; ++x) {
    for (int y = 0; y < 2 * 16 - 1; ++y) {
      _LUT.at<float>(y, x) = std::max(
          1 - static_cast<double>((15 - x) * (15 - x) + (15 - y) * (15 - y))
                  / static_cast<double>(15 * 15),
          0.0);
    }
  }
}

template<class SCORE_CALCULATOR_T>
ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScaleSpaceLayer(
    ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores) {
  Create(layerBelow, initScores);
}

template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::Create(
    ScaleSpaceLayer<ScoreCalculator_t>* layerBelow, bool initScores) {
  // For successive construction.
  /*brisk::timing::Timer timerDownsample(
      "0.0 BRISK Detection: Creation&Downsampling (per layer)");*/
  int type = layerBelow->_img.type();
  if (layerBelow->_isOctave) {
    if (layerBelow->_layerNumber >= 2) {
      // We can do the (cheaper) halfsampling.
      _img.create(layerBelow->_belowLayer_ptr->_img.rows / 2,
                  layerBelow->_belowLayer_ptr->_img.cols / 2, type);
      Halfsample(layerBelow->_belowLayer_ptr->_img, _img);

    } else {
      // We do the two-third sampling.
      _img.create((layerBelow->_img.rows / 3) * 2,
                  (layerBelow->_img.cols / 3) * 2, type);
      Twothirdsample(layerBelow->_img, _img);
    }
    // Keep track of where in the pyramid we are.
    _isOctave = false;
  } else {
    // We can do the (cheaper) halfsampling.
    _img.create(layerBelow->_belowLayer_ptr->_img.rows / 2,
                layerBelow->_belowLayer_ptr->_img.cols / 2, type);
    Halfsample(layerBelow->_belowLayer_ptr->_img, _img);

    // Keep track of where in the pyramid we are.
    _isOctave = true;
  }
  // Keep track.
  _layerNumber = layerBelow->_layerNumber + 1;
  _belowLayer_ptr = layerBelow;
  layerBelow->_aboveLayer_ptr = this;

  // Calculate coordinate transformation parameters.
  if (_isOctave) {
    _offset_above = -0.25;
    _offset_below = 1.0 / 6.0;
    _scale_above = 2.0 / 3.0;
    _scale_below = 4.0 / 3.0;
    _scale = pow(2.0, static_cast<double>(_layerNumber / 2));
    _offset = _scale * 0.5 - 0.5;
  } else {
    _offset_above = -1.0 / 6.0;
    _offset_below = 0.125;
    _scale_above = 0.75;
    _scale_below = 1.5;
    _scale = pow(2.0, static_cast<double>(_layerNumber / 2)) * 1.5;
    _offset = _scale * 0.5 - 0.5;
  }
  //timerDownsample.Stop();

  // By default no uniformity radius.
  _radius = 1;

  // Abs. threshold (for noise rejection).
  _absoluteThreshold = 0;

  // Initialize the score calculation.
  _scoreCalculator.SetImage(_img, initScores);

  // The above layer is undefined:
  _aboveLayer_ptr = 0;

  // Generic mask.
  _LUT = cv::Mat::zeros(2 * 16 - 1, 2 * 16 - 1, CV_32F);
  for (int x = 0; x < 2 * 16 - 1; ++x) {
    for (int y = 0; y < 2 * 16 - 1; ++y) {
      _LUT.at<float>(y, x) = std::max(
          1 - static_cast<double>((15 - x) * (15 - x) + (15 - y) * (15 - y))
                  / static_cast<double>(15 * 15),
          0.0);
    }
  }
}

template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::SetUniformityRadius(double radius) {
  _radius = radius;
  if (radius == 0)
    _radius = 1;
}

// Feature detection.
template<class SCORE_CALCULATOR_T>
void ScaleSpaceLayer<SCORE_CALCULATOR_T>::DetectScaleSpaceMaxima(
    std::vector<cv::KeyPoint>& keypoints, bool enforceUniformity,
    bool doRefinement, bool usePassedKeypoints) {
  // First get the maxima points inside this layer.
  std::vector<typename ScoreCalculator_t::PointWithScore> points;
  if (usePassedKeypoints) {
    points.reserve(keypoints.size());
    for (size_t k = 0; k < keypoints.size(); ++k) {
      if (cv::KeyPointResponse(keypoints[k]) > 1e6) {
        points.push_back(
            typename ScoreCalculator_t::PointWithScore(
                cv::KeyPointResponse(keypoints[k]),
                cv::KeyPointX(keypoints[k]),
                cv::KeyPointY(keypoints[k])));
      }
    }
  } else {
    /*brisk::timing::DebugTimer timerNonMaxSuppression2d(
        "0.2 BRISK Detection: 2d nonmax suppression (per layer)");*/
    _scoreCalculator.Get2dMaxima(points, _absoluteThreshold);
    //timerNonMaxSuppression2d.Stop();
  }
  // Next check above and below. The code looks a bit stupid, but that's
  // for speed. We don't want to make the distinction analyzing whether or
  // not there is a layer above and below inside the loop.
  if (!usePassedKeypoints) {
    if (_aboveLayer_ptr != 0 && _belowLayer_ptr != 0) {
      // Check above and below
      /*brisk::timing::DebugTimer timerNonMaxSuppression3d(
          "0.3 BRISK Detection: 3d nonmax suppression (per layer)");*/
      std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
      pt_tmp.reserve(points.size());
      const int one_over_scale_above = 1.0 / _scale_above;
      const int one_over_scale_below = 1.0 / _scale_below;
      for (typename std::vector<
          typename ScoreCalculator_t::PointWithScore>::const_iterator it =
          points.begin(); it != points.end(); ++it) {
        const typename ScoreCalculator_t::Score_t center = it->score;
        const int x = it->x;
        const int y = it->y;
        if (center < (typename ScoreCalculator_t::Score_t) (_absoluteThreshold))
          continue;
        if (center < ScoreAbove(x, y))
          continue;
        if (center < ScoreAbove(x + one_over_scale_above, y))
          continue;
        if (center < ScoreAbove(x - one_over_scale_above, y))
          continue;
        if (center < ScoreAbove(x, y + one_over_scale_above))
          continue;
        if (center < ScoreAbove(x, y - one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x + one_over_scale_above, y + one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x + one_over_scale_above, y - one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x - one_over_scale_above, y + one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x - one_over_scale_above, y - one_over_scale_above))
          continue;
        if (center < ScoreBelow(x, y))
          continue;
        if (center < ScoreBelow(x + one_over_scale_below, y))
          continue;
        if (center < ScoreBelow(x - one_over_scale_below, y))
          continue;
        if (center < ScoreBelow(x, y + one_over_scale_below))
          continue;
        if (center < ScoreBelow(x, y - one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x + one_over_scale_below, y + one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x + one_over_scale_below, y - one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x - one_over_scale_below, y + one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x - one_over_scale_below, y - one_over_scale_below))
          continue;
        pt_tmp.push_back(*it);
      }
      points.assign(pt_tmp.begin(), pt_tmp.end());
      //timerNonMaxSuppression3d.Stop();
    } else if (_aboveLayer_ptr != 0) {
      // Check above.
      /*brisk::timing::DebugTimer timerNonMaxSuppression3d(
          "0.3 BRISK Detection: 3d nonmax suppression (per layer)");*/
      std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
      pt_tmp.reserve(points.size());
      const int one_over_scale_above = 1.0 / _scale_above;
      for (typename std::vector<
          typename ScoreCalculator_t::PointWithScore>::const_iterator it =
          points.begin(); it != points.end(); ++it) {
        const typename ScoreCalculator_t::Score_t center = it->score;
        if (center < (typename ScoreCalculator_t::Score_t) (_absoluteThreshold))
          continue;
        const int x = it->x;
        const int y = it->y;
        if (center < ScoreAbove(x, y))
          continue;
        if (center < ScoreAbove(x + one_over_scale_above, y))
          continue;
        if (center < ScoreAbove(x - one_over_scale_above, y))
          continue;
        if (center < ScoreAbove(x, y + one_over_scale_above))
          continue;
        if (center < ScoreAbove(x, y - one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x + one_over_scale_above, y + one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x + one_over_scale_above, y - one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x - one_over_scale_above, y + one_over_scale_above))
          continue;
        if (center
            < ScoreAbove(x - one_over_scale_above, y - one_over_scale_above))
          continue;
        pt_tmp.push_back(*it);
      }
      points.assign(pt_tmp.begin(), pt_tmp.end());
      //timerNonMaxSuppression3d.Stop();
    } else if (_belowLayer_ptr != 0) {
      // Check below.
      /*brisk::timing::DebugTimer timerNonMaxSuppression3d(
          "0.3 BRISK Detection: 3d nonmax suppression (per layer)");*/
      std::vector<typename ScoreCalculator_t::PointWithScore> pt_tmp;
      pt_tmp.reserve(points.size());
      const int one_over_scale_below = 1.0 / _scale_below;
      for (typename std::vector<
          typename ScoreCalculator_t::PointWithScore>::const_iterator it =
          points.begin(); it != points.end(); ++it) {
        const typename ScoreCalculator_t::Score_t center = it->score;
        if (center < (typename ScoreCalculator_t::Score_t) (_absoluteThreshold))
          continue;
        const int x = it->x;
        const int y = it->y;
        if (center < ScoreBelow(x, y))
          continue;
        if (center < ScoreBelow(x + one_over_scale_below, y))
          continue;
        if (center < ScoreBelow(x - one_over_scale_below, y))
          continue;
        if (center < ScoreBelow(x, y + one_over_scale_below))
          continue;
        if (center < ScoreBelow(x, y - one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x + one_over_scale_below, y + one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x + one_over_scale_below, y - one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x - one_over_scale_below, y + one_over_scale_below))
          continue;
        if (center
            < ScoreBelow(x - one_over_scale_below, y - one_over_scale_below))
          continue;
        pt_tmp.push_back(*it);
      }
      points.assign(pt_tmp.begin(), pt_tmp.end());
      //timerNonMaxSuppression3d.Stop();
    }
  }

  // Uniformity enforcement.
  if (points.size() == 0)
    return;
  if (enforceUniformity && _radius > 0.0) {
    EnforceKeyPointUniformity(_LUT, _radius, _img.rows, _img.cols, _maxNumKpt,
                              points);
  }

  // 3d(/2d) subpixel refinement.
  /*brisk::timing::DebugTimer timer_subpixel_refinement(
      "0.4 BRISK Detection: "
      "subpixel(&scale) refinement (per layer)");*/
  if (usePassedKeypoints)
    keypoints.clear();
  if (doRefinement) {
    for (typename std::vector<
        typename ScoreCalculator_t::PointWithScore>::const_iterator it =
        points.begin(); it != points.end(); ++it) {
      const int u = it->x;
      const int v = it->y;
      float delta_x;
      float delta_y;
      Subpixel2D(_scoreCalculator.Score(u - 1, v - 1),
                 _scoreCalculator.Score(u, v - 1),
                 _scoreCalculator.Score(u + 1, v - 1),
                 _scoreCalculator.Score(u - 1, v),
                 _scoreCalculator.Score(u, v),
                 _scoreCalculator.Score(u + 1, v),
                 _scoreCalculator.Score(u - 1, v + 1),
                 _scoreCalculator.Score(u, v + 1),
                 _scoreCalculator.Score(u + 1, v + 1), delta_x, delta_y);
      // TODO(lestefan): 3d refinement.
      cv::KeyPoint keypoint;
      cv::KeyPointX(keypoint) = _scale * ((it->x + delta_x) + _offset);
      cv::KeyPointY(keypoint) = _scale * ((it->y + delta_y) + _offset);
      cv::KeyPointSize(keypoint) = _scale * 12.0;
      cv::KeyPointAngle(keypoint) = -1;
      cv::KeyPointResponse(keypoint) = it->score;
      cv::KeyPointOctave(keypoint) = _layerNumber / 2;
      keypoints.push_back(keypoint);
    }
  } else {
    for (typename std::vector<
        typename ScoreCalculator_t::PointWithScore>::const_iterator it =
        points.begin(); it != points.end(); ++it) {
      cv::KeyPoint keypoint;
      cv::KeyPointX(keypoint) = _scale * ((it->x) + _offset);
      cv::KeyPointY(keypoint) = _scale * ((it->y) + _offset);
      cv::KeyPointSize(keypoint) = _scale * 12.0;
      cv::KeyPointAngle(keypoint) = -1;
      cv::KeyPointResponse(keypoint) = it->score;
      cv::KeyPointOctave(keypoint) = _layerNumber / 2;
      keypoints.push_back(keypoint);
    }
  }
  //timer_subpixel_refinement.Stop();
}

// Utilities.
template<class SCORE_CALCULATOR_T>
inline double ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScoreAbove(double u,
                                                              double v) {
  return _aboveLayer_ptr->_scoreCalculator.Score(
      _scale_above * (u + _offset_above), _scale_above * (v + _offset_above));
}
template<class SCORE_CALCULATOR_T>
inline double ScaleSpaceLayer<SCORE_CALCULATOR_T>::ScoreBelow(double u,
                                                              double v) {
  return _belowLayer_ptr->_scoreCalculator.Score(
      _scale_below * (u + _offset_below), _scale_below * (v + _offset_below));
}

template<class SCORE_CALCULATOR_T>
inline bool ScaleSpaceLayer<SCORE_CALCULATOR_T>::Halfsample(
    const cv::Mat& srcimg, cv::Mat& dstimg) {
  if (srcimg.type() == CV_8UC1) {
    Halfsample8(srcimg, dstimg);
  } else if (srcimg.type() == CV_16UC1) {
    Halfsample16(srcimg, dstimg);
  } else {
    return false;
  }
  return true;
}

template<class SCORE_CALCULATOR_T>
inline bool ScaleSpaceLayer<SCORE_CALCULATOR_T>::Twothirdsample(
    const cv::Mat& srcimg, cv::Mat& dstimg) {
  if (srcimg.type() == CV_8UC1) {
    Twothirdsample8(srcimg, dstimg);
  } else if (srcimg.type() == CV_16UC1) {
    Twothirdsample16(srcimg, dstimg);
  } else {
    return false;
  }
  return true;
}

template<class SCORE_CALCULATOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULATOR_T>::Refine1D(const float s_05,
                                                               const float s0,
                                                               const float s05,
                                                               float& max) {
  int i_05 = static_cast<int>(1024.0 * s_05 + 0.5);
  int i0 = static_cast<int>(1024.0 * s0 + 0.5);
  int i05 = static_cast<int>(1024.0 * s05 + 0.5);

  //   16.0000  -24.0000    8.0000
  //  -40.0000   54.0000  -14.0000
  //   24.0000  -27.0000    6.0000

  int three_a = 16 * i_05 - 24 * i0 + 8 * i05;
  // Second derivative must be negative:
  if (three_a >= 0) {
    if (s0 >= s_05 && s0 >= s05) {
      max = s0;
      return 1.0;
    }
    if (s_05 >= s0 && s_05 >= s05) {
      max = s_05;
      return 0.75;
    }
    if (s05 >= s0 && s05 >= s_05) {
      max = s05;
      return 1.5;
    }
  }

  int three_b = -40 * i_05 + 54 * i0 - 14 * i05;
  // Calculate max location:
  float ret_val = -static_cast<float>(three_b) /
      static_cast<float>(2 * three_a);
  // Saturate and return
  if (ret_val < 0.75)
    ret_val = 0.75;
  else if (ret_val > 1.5)
    ret_val = 1.5;  // Allow to be slightly off bounds ...?
  int three_c = +24 * i_05 - 27 * i0 + 6 * i05;
  max = static_cast<float>(three_c) + static_cast<float>(three_a) *
      ret_val * ret_val + static_cast<float>(three_b) * ret_val;
  max /= 3072.0;
  return ret_val;
}

template<class SCORE_CALCULATOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULATOR_T>::Refine1D_1(
    const float s_05, const float s0, const float s05, float& max) {
  int i_05 = static_cast<int>(1024.0 * s_05 + 0.5);
  int i0 = static_cast<int>(1024.0 * s0 + 0.5);
  int i05 = static_cast<int>(1024.0 * s05 + 0.5);

  //  4.5000   -9.0000    4.5000
  // -10.5000   18.0000   -7.5000
  //  6.0000   -8.0000    3.0000

  int two_a = 9 * i_05 - 18 * i0 + 9 * i05;
  // Second derivative must be negative:
  if (two_a >= 0) {
    if (s0 >= s_05 && s0 >= s05) {
      max = s0;
      return 1.0;
    }
    if (s_05 >= s0 && s_05 >= s05) {
      max = s_05;
      return 0.6666666666666666666666666667;
    }
    if (s05 >= s0 && s05 >= s_05) {
      max = s05;
      return 1.3333333333333333333333333333;
    }
  }

  int two_b = -21 * i_05 + 36 * i0 - 15 * i05;
  // calculate max location:
  float ret_val = -static_cast<float>(two_b) / static_cast<float>(2 * two_a);
  // saturate and return
  if (ret_val < 0.6666666666666666666666666667)
    ret_val = 0.666666666666666666666666667;
  else if (ret_val > 1.33333333333333333333333333)
    ret_val = 1.333333333333333333333333333;
  int two_c = +12 * i_05 - 16 * i0 + 6 * i05;
  max = static_cast<float>(two_c) + static_cast<float>(two_a) * ret_val *
      ret_val + static_cast<float>(two_b) * ret_val;
  max /= 2048.0;
  return ret_val;
}

template<class SCORE_CALCULATOR_T>
__inline__ float ScaleSpaceLayer<SCORE_CALCULATOR_T>::Subpixel2D(
    const double s_0_0, const double s_0_1, const double s_0_2,
    const double s_1_0, const double s_1_1, const double s_1_2,
    const double s_2_0, const double s_2_1, const double s_2_2, float& delta_x,
    float& delta_y) {
  // The coefficients of the 2d quadratic function least-squares fit:
  double tmp1 = s_0_0 + s_0_2 - 2 * s_1_1 + s_2_0 + s_2_2;
  double coeff1 = 3 * (tmp1 + s_0_1 - ((s_1_0 + s_1_2) / 2.0) + s_2_1);
  double coeff2 = 3 * (tmp1 - ((s_0_1 + s_2_1) / 2.0) + s_1_0 + s_1_2);
  double tmp2 = s_0_2 - s_2_0;
  double tmp3 = (s_0_0 + tmp2 - s_2_2);
  double tmp4 = tmp3 - 2 * tmp2;
  double coeff3 = -3 * (tmp3 + s_0_1 - s_2_1);
  double coeff4 = -3 * (tmp4 + s_1_0 - s_1_2);
  double coeff5 = (s_0_0 - s_0_2 - s_2_0 + s_2_2) / 4.0;
  double coeff6 = -(s_0_0 + s_0_2
      - ((s_1_0 + s_0_1 + s_1_2 + s_2_1) / 2.0) - 5 * s_1_1 + s_2_0 + s_2_2)
      / 2.01;

  // 2nd derivative test:
  double H_det = 4 * coeff1 * coeff2 - coeff5 * coeff5;

  if (H_det == 0) {
    delta_x = 0.0;
    delta_y = 0.0;
    return static_cast<float>(coeff6) / 18.0;
  }

  if (!(H_det > 0 && coeff1 < 0)) {
    // The maximum must be at the one of the 4 patch corners.
    int tmp_max = coeff3 + coeff4 + coeff5;
    delta_x = 1.0;
    delta_y = 1.0;

    int tmp = -coeff3 + coeff4 - coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = -1.0;
      delta_y = 1.0;
    }
    tmp = coeff3 - coeff4 - coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = 1.0;
      delta_y = -1.0;
    }
    tmp = -coeff3 - coeff4 + coeff5;
    if (tmp > tmp_max) {
      tmp_max = tmp;
      delta_x = -1.0;
      delta_y = -1.0;
    }
    return static_cast<float>(tmp_max + coeff1 + coeff2 + coeff6) / 18.0;
  }

  // This is hopefully the normal outcome of the Hessian test.
  delta_x = static_cast<float>(2 * coeff2 * coeff3 - coeff4 * coeff5) /
      static_cast<float>(-H_det);
  delta_y = static_cast<float>(2 * coeff1 * coeff4 - coeff3 * coeff5) /
      static_cast<float>(-H_det);
  // TODO(lestefan): this is not correct, but easy, so perform a real boundary
  // maximum search:
  bool tx = false;
  bool tx_ = false;
  bool ty = false;
  bool ty_ = false;
  if (delta_x > 1.0)
    tx = true;
  else if (delta_x < -1.0)
    tx_ = true;
  if (delta_y > 1.0)
    ty = true;
  if (delta_y < -1.0)
    ty_ = true;

  if (tx || tx_ || ty || ty_) {
    // Get two candidates:
    float delta_x1 = 0.0, delta_x2 = 0.0, delta_y1 = 0.0, delta_y2 = 0.0;
    if (tx) {
      delta_x1 = 1.0;
      delta_y1 = -static_cast<float>(coeff4 + coeff5) /
          static_cast<float>(2 * coeff2);
      if (delta_y1 > 1.0)
        delta_y1 = 1.0;
      else if (delta_y1 < -1.0)
        delta_y1 = -1.0;
    } else if (tx_) {
      delta_x1 = -1.0;
      delta_y1 = -static_cast<float>(coeff4 - coeff5) /
          static_cast<float>(2 * coeff2);
      if (delta_y1 > 1.0)
        delta_y1 = 1.0;
      else if (delta_y1 < -1.0)
        delta_y1 = -1.0;
    }
    if (ty) {
      delta_y2 = 1.0;
      delta_x2 = -static_cast<float>(coeff3 + coeff5) /
          static_cast<float>(2 * coeff1);
      if (delta_x2 > 1.0)
        delta_x2 = 1.0;
      else if (delta_x2 < -1.0)
        delta_x2 = -1.0;
    } else if (ty_) {
      delta_y2 = -1.0;
      delta_x2 = -static_cast<float>(coeff3 - coeff5) /
          static_cast<float>(2 * coeff1);
      if (delta_x2 > 1.0)
        delta_x2 = 1.0;
      else if (delta_x2 < -1.0)
        delta_x2 = -1.0;
    }
    // Insert both options for evaluation which to pick.
    float max1 = (coeff1 * delta_x1 * delta_x1 + coeff2 * delta_y1 * delta_y1
        + coeff3 * delta_x1 + coeff4 * delta_y1 + coeff5 * delta_x1 * delta_y1
        + coeff6) / 18.0;
    float max2 = (coeff1 * delta_x2 * delta_x2 + coeff2 * delta_y2 * delta_y2
        + coeff3 * delta_x2 + coeff4 * delta_y2 + coeff5 * delta_x2 * delta_y2
        + coeff6) / 18.0;
    if (max1 > max2) {
      delta_x = delta_x1;
      delta_y = delta_x1;
      return max1;
    } else {
      delta_x = delta_x2;
      delta_y = delta_x2;
      return max2;
    }
  }
  // This is the case of the maximum inside the boundaries:
  return (coeff1 * delta_x * delta_x + coeff2 * delta_y * delta_y
      + coeff3 * delta_x + coeff4 * delta_y + coeff5 * delta_x * delta_y
      + coeff6) / 18.0;
}
}  // namespace brisk
#endif  // INTERNAL_SCALE_SPACE_LAYER_INL_H_
