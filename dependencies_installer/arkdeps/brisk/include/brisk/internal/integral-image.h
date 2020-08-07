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

#ifndef INTERNAL_INTEGRAL_IMAGE_H_
#define INTERNAL_INTEGRAL_IMAGE_H_

#ifdef __ARM_NEON__
#include <arm_neon.h>
#else
#include <emmintrin.h>
#include <pmmintrin.h>
#include <tmmintrin.h>
#endif  // __ARM_NEON__

#include <stdexcept>


namespace brisk {
void IntegralImage8(const cv::Mat& src, cv::Mat* dest) {
  if(dest==nullptr){
    throw std::runtime_error("destination image NULL");
  }
  int x, y;
  const int cn = 1;
  const int srcstep = static_cast<int>(src.step / sizeof(unsigned char));

  dest->create(src.rows + 1, src.cols + 1, CV_MAKETYPE(CV_32S, 1));

  const int sumstep = static_cast<int>(dest->step / sizeof(int32_t));

  int* sum = reinterpret_cast<int32_t*>(dest->data);
  unsigned char* _src = static_cast<unsigned char*>(src.data);

  memset(sum, 0, (src.cols + cn) * sizeof(sum[0]));
  sum += sumstep + 1;

  const int srcstep_2 = 2 * srcstep;
  const int sumstep_2 = 2 * sumstep;

  const int maxRows = src.rows - 2;
  const int maxCols = src.cols - 4;

  for (y = 0; y <= maxRows; y += 2, _src += srcstep_2, sum += sumstep_2) {
    int s = sum[-1] = 0;
    int ss = sum[sumstep - 1] = 0;
    for (x = 0; x <= maxCols; x += 4) {
      const int x1 = x + 1;
      const int x2 = x + 2;
      const int x3 = x + 3;
      const int s0 = s + _src[x];
      const int s1 = s0 + _src[x1];
      const int s2 = s1 + _src[x2];
      const int s3 = s2 + _src[x3];
      s = s3;
#ifdef __ARM_NEON__
      int tmp1[4] = {s0, s1, s2, s3};
      int32x4_t  __src0 = vld1q_s32(tmp1);
      int32x4_t    __sum = vld1q_s32(&sum[x - sumstep]);
      __sum = vaddq_s32(__src0, __sum);
      vst1q_s32(&sum[x], __sum);
#else
      __m128i    __src0 = _mm_set_epi32(s3, s2, s1, s0);
      __m128i    __sum =
          _mm_lddqu_si128(reinterpret_cast<__m128i *>(&sum[x - sumstep]));
      __sum = _mm_add_epi32(__src0, __sum);
      _mm_storeu_si128(reinterpret_cast<__m128i *>(&sum[x]), __sum);
#endif

      const unsigned char * __src = _src + srcstep;
      const int s0_1 = ss + __src[x];
      const int s1_1 = s0_1 + __src[x1];
      const int s2_1 = s1_1 + __src[x2];
      const int s3_1 = s2_1 + __src[x3];
      ss = s3_1;
#ifdef __ARM_NEON__
      int tmp2[4] = {s0_1, s1_1, s2_1, s3_1};
      __src0 = vld1q_s32(tmp2);
      __sum = vaddq_s32(__src0, __sum);
      vst1q_s32(&sum[x + sumstep], __sum);
#else
      __src0 = _mm_set_epi32(s3_1, s2_1, s1_1, s0_1);
      __sum = _mm_add_epi32(__src0, __sum);
      _mm_storeu_si128(reinterpret_cast<__m128i *>(&sum[x + sumstep]), __sum);
#endif
    }

    // Finish the row.
    for (int _x = x; _x < src.cols; ++_x) {
      s += _src[_x];
      sum[_x] = sum[_x - sumstep] + s;
      ss += _src[_x + srcstep];
      sum[_x + sumstep] = sum[_x] + ss;
    }
  }

  // Remaining row.
  for (int _y = y; _y < src.rows; _y++, _src += srcstep, sum += sumstep) {
    int s = sum[-1] = 0;
    for (x = 0; x <= maxCols; x += 4) {
      const int s0 = s + _src[x];
      const int s1 = s0 + _src[x + 1];
      const int s2 = s1 + _src[x + 2];
      const int s3 = s2 + _src[x + 3];
      s = s3;
#ifdef __ARM_NEON__
      int tmp3[4] = {s0, s1, s2, s3};
      int32x4_t __src0 = vld1q_s32(tmp3);
      int32x4_t  __sum = vld1q_s32(&sum[x - sumstep]);
      __sum = vaddq_s32(__src0, __sum);
      vst1q_s32(&sum[x], __sum);
#else
      __m128i     __src0 = _mm_set_epi32(s3, s2, s1, s0);
      __m128i     __sum =
          _mm_lddqu_si128(reinterpret_cast<__m128i *>(&sum[x - sumstep]));
      __sum = _mm_add_epi32(__src0, __sum);
      _mm_storeu_si128(reinterpret_cast<__m128i *>(&sum[x]), __sum);
#endif
    }

    // Finish the row.
    for (int _x = x; _x < src.cols; ++_x) {
      s += _src[_x];
      sum[_x] = sum[_x - sumstep] + s;
    }
  }
}

void IntegralImage16(const cv::Mat& src, cv::Mat* dest) {
  if(dest==nullptr){
    throw std::runtime_error("destination image NULL");
  }
  int x, y;
  const int cn = 1;
  const int srcstep = static_cast<int>(src.step / sizeof(uint16_t));

  dest->create(src.rows + 1, src.cols + 1, CV_MAKETYPE(CV_32F, 1));

  float float_val;
  const int sumstep = static_cast<int>(dest->step / sizeof(float_val));

  float* sum = reinterpret_cast<float*>(dest->data);
  uint16_t* _src = reinterpret_cast<uint16_t*>(src.data);

  memset(sum, 0, (src.cols + cn) * sizeof(sum[0]));
  sum += sumstep + 1;

  const int maxRows = src.rows;
  const int maxCols = src.cols - 4;
  const float cvtScale = 1.0 / 65536.0;

  for (y = 0; y < maxRows; y++, _src += srcstep, sum += sumstep) {
    float s = sum[-1] = 0.0;
    for (x = 0; x <= maxCols; x += 4) {
      const int x1 = x + 1;
      const int x2 = x + 2;
      const int x3 = x + 3;

      float s0 = s + _src[x] * cvtScale;
      float s1 = s0 + _src[x1] * cvtScale;
      float s2 = s1 + _src[x2] * cvtScale;
      float s3 = s2 + _src[x3] * cvtScale;
      s = s3;
#ifdef __ARM_NEON__
      float tmp1[4] = {s0, s1, s2, s3};
      float32x4_t  __src0 = vld1q_f32(tmp1);

      float32x4_t  __sum = vld1q_f32(&sum[x - sumstep]);
      __sum = vaddq_f32(__src0, __sum);
      vst1q_f32(&sum[x], __sum);
#else
      __m128  __src0 = _mm_set_ps(s3, s2, s1, s0);

      __m128  __sum = _mm_loadu_ps(&sum[x - sumstep]);
      __sum = _mm_add_ps(__src0, __sum);
      _mm_storeu_ps(&sum[x], __sum);
#endif
    }

    // Finish the row.
    for (int _x = x; _x < src.cols; ++_x) {
      s += _src[_x];
      sum[_x] = sum[_x - sumstep] + s;
    }
  }
}
}  // namespace brisk
#endif  // INTERNAL_INTEGRAL_IMAGE_H_
