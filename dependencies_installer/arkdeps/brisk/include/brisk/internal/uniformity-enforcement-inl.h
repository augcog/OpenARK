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

#ifndef BRISK_UNIFORMITY_ENFORCEMENT_INL_H_
#define BRISK_UNIFORMITY_ENFORCEMENT_INL_H_


//#include <brisk/internal/timer.h>

template<typename POINT_WITH_SCORE>
void EnforceKeyPointUniformity(const cv::Mat& LUT, double radius,
                               int imgrows, int imgcols, size_t maxNumKpt,
                               std::vector<POINT_WITH_SCORE>& points) {
  /*brisk::timing::DebugTimer timer_sort_keypoints(
      "0.31 BRISK Detection: "
      "sort keypoints by score (per layer)");*/
  std::vector<POINT_WITH_SCORE> pt_tmp;

  // Sort.
  std::sort(points.begin(), points.end());
  const float maxScore = points.front().score;
  //timer_sort_keypoints.Stop();

  pt_tmp.reserve(points.size());  // Allow appending.

      // Store occupancy.
  cv::Mat occupancy;
  const float scaling = 15.0 / static_cast<float>(radius);
  occupancy = cv::Mat::zeros((imgrows) * ceil(scaling) + 32,
                             (imgcols) * ceil(scaling) + 32, CV_8U);

  /*brisk::timing::DebugTimer timer_uniformity_enforcement(
      "0.3 BRISK Detection: "
      "uniformity enforcement (per layer)");*/
  // Go through the sorted keypoints and reject too close ones.
  for (typename std::vector<POINT_WITH_SCORE>::const_iterator it =
      points.begin(); it != points.end(); ++it) {
    const int cy = (it->y * scaling + 16);
    const int cx = (it->x * scaling + 16);

    // Check if this is a high enough score.
    const double s0 = static_cast<double>(occupancy.at<unsigned char>(cy, cx));
    const float nsc1 = sqrtf(sqrtf(it->score / maxScore)) * 255.0f;

    if (nsc1 < s0)
      continue;

    // Masks.
    const float nsc = 0.99f * nsc1;
    for (int y = 0; y < 2 * 16 - 1; ++y) {
#ifdef __ARM_NEON__
      uint8x16_t mem1 = vld1q_u8(reinterpret_cast<const uint8_t*>(
              &occupancy.at<uint8_t>(cy + y - 15, cx - 15)));
      uint8x16_t mem2 = vld1q_u8(reinterpret_cast<const uint8_t*>(
              &occupancy.at<uint8_t>(cy + y - 15, cx + 1)));

      const uint8_t tmpstore_mask1[16] = {
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 0) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 1) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 2) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 3) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 4) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 5) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 6) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 7) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 8) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 9) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 10) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 11) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 12) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 13) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 14) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 15) * nsc))};
      // Lacking the masked storing intrinsics in NEON.
      uint8x16_t mask1 = vld1q_u8(tmpstore_mask1);

      const uint8_t tmpstore_mask2[16] = {
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 16) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 17) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 18) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 19) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 20) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 21) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 22) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 23) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 24) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 25) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 26) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 27) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 28) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 29) * nsc)),
        static_cast<uint8_t>(ceil(LUT.at<float>(y, 30) * nsc)),
        0};
      // Lacking the masked storing intrinsics in NEON.
      uint8x16_t mask2 = vld1q_u8(tmpstore_mask2);

      vst1q_u8(&occupancy.at<uint8_t>(cy + y - 15, cx - 15),
          vqaddq_u8(mem1, mask1));
      vst1q_u8(&occupancy.at<uint8_t>(cy + y - 15, cx + 1),
          vqaddq_u8(mem2, mask2));
# else
      __m128i mem1 =
          _mm_loadu_si128(
              reinterpret_cast<__m128i *>(&occupancy.at<unsigned char>(cy + y - 15,
                                                               cx - 15)));
      __m128i mem2 =
          _mm_loadu_si128(
              reinterpret_cast<__m128i *>(&occupancy.at<unsigned char>(cy + y - 15,
                                                               cx + 1)));
      __m128i mask1 = _mm_set_epi8(ceil(LUT.at<float>(y, 15) * nsc),
                                   ceil(LUT.at<float>(y, 14) * nsc),
                                   ceil(LUT.at<float>(y, 13) * nsc),
                                   ceil(LUT.at<float>(y, 12) * nsc),
                                   ceil(LUT.at<float>(y, 11) * nsc),
                                   ceil(LUT.at<float>(y, 10) * nsc),
                                   ceil(LUT.at<float>(y, 9) * nsc),
                                   ceil(LUT.at<float>(y, 8) * nsc),
                                   ceil(LUT.at<float>(y, 7) * nsc),
                                   ceil(LUT.at<float>(y, 6) * nsc),
                                   ceil(LUT.at<float>(y, 5) * nsc),
                                   ceil(LUT.at<float>(y, 4) * nsc),
                                   ceil(LUT.at<float>(y, 3) * nsc),
                                   ceil(LUT.at<float>(y, 2) * nsc),
                                   ceil(LUT.at<float>(y, 1) * nsc),
                                   ceil(LUT.at<float>(y, 0) * nsc));
      __m128i mask2 = _mm_set_epi8(0, ceil(LUT.at<float>(y, 30) * nsc),
                                   ceil(LUT.at<float>(y, 29) * nsc),
                                   ceil(LUT.at<float>(y, 28) * nsc),
                                   ceil(LUT.at<float>(y, 27) * nsc),
                                   ceil(LUT.at<float>(y, 26) * nsc),
                                   ceil(LUT.at<float>(y, 25) * nsc),
                                   ceil(LUT.at<float>(y, 24) * nsc),
                                   ceil(LUT.at<float>(y, 23) * nsc),
                                   ceil(LUT.at<float>(y, 22) * nsc),
                                   ceil(LUT.at<float>(y, 21) * nsc),
                                   ceil(LUT.at<float>(y, 20) * nsc),
                                   ceil(LUT.at<float>(y, 19) * nsc),
                                   ceil(LUT.at<float>(y, 18) * nsc),
                                   ceil(LUT.at<float>(y, 17) * nsc),
                                   ceil(LUT.at<float>(y, 16) * nsc));
      _mm_storeu_si128(
          reinterpret_cast<__m128i *>(&occupancy.at<unsigned char>(cy + y - 15, cx - 15)),
          _mm_adds_epu8(mem1, mask1));
      _mm_storeu_si128(
          reinterpret_cast<__m128i *>(&occupancy.at<unsigned char>(cy + y - 15, cx + 1)),
          _mm_adds_epu8(mem2, mask2));
#endif  // __ARM_NEON__
    }

    // Store.
    pt_tmp.push_back(*it);

    if (pt_tmp.size() == maxNumKpt) {
      break;
    }  // Limit the max number if necessary.
  }
  points.assign(pt_tmp.begin(), pt_tmp.end());

  //timer_uniformity_enforcement.Stop();
}
#endif  // BRISK_UNIFORMITY_ENFORCEMENT_INL_H_
