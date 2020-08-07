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

#ifndef INTERNAL_VECTORIZED_FILTERS_INL_H_
#define INTERNAL_VECTORIZED_FILTERS_INL_H_

#ifdef __ARM_NEON__
#include <arm_neon.h>
#else
#include <emmintrin.h>
#include <tmmintrin.h>
#endif  // __ARM_NEON__
#include <stdint.h>



template<int X, int Y>
__inline__ void Filter2D16S(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel) {  // NOLINT
  // Sanity check.
  assert(kernel.type() == CV_16S);
  assert(Y == kernel.rows);
  assert(X == kernel.cols);
  assert(X % 2 != 0);
  assert(Y % 2 != 0);
  int cx = X / 2;
  int cy = Y / 2;

  // Destination will be 16 bit.
  dst = cv::Mat::zeros(src.rows, src.cols, CV_16S);
  const unsigned int maxJ = ((src.cols - 2) / 8) * 8;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      __m128i result = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      // Enter convolution with kernel.
      for (unsigned int x = 0; x < X; ++x) {
        for (unsigned int y = 0; y < Y; ++y) {
          const char m = kernel.at<int16_t>(y, x);
          if (m == 0)
            continue;
          __m128i mult = _mm_set_epi16(m, m, m, m, m, m, m, m);
          __m128i i0 = _mm_loadu_si128(
              reinterpret_cast<__m128i *>(&src.at<int16_t>(i + y, j + x)));
          __m128i i1 = _mm_mullo_epi16(i0, mult);
          result = _mm_add_epi16(result, i1);
        }
      }
      // Store
      _mm_storeu_si128(reinterpret_cast<__m128i *>(
          &dst.at<int16_t>(i + cy, j + cx)), result);

      // Take care of end.
      j += 8;
      if (j >= maxJ && !end) {
        j = stride - 2 - 8;
        end = true;
      }
    }
  }
}

template<int X, int Y>
__inline__ void Filter2D8U(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel) {  // NOLINT
  // Sanity check.
  assert(kernel.type() == CV_8S);
  assert(Y == kernel.rows);
  assert(X == kernel.cols);
  assert(X % 2 != 0);
  assert(Y % 2 != 0);
  int cx = X / 2;
  int cy = Y / 2;

  // Destination will be 16 bit.
  dst = cv::Mat::zeros(src.rows, src.cols, CV_16S);
  const unsigned int maxJ = ((src.cols - 2) / 16) * 16;
  const unsigned int maxI = src.rows - 2;
  const unsigned int stride = src.cols;

  __m128i mask_hi = _mm_set_epi8(0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                                 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
                                 0xFF);
  __m128i mask_lo = _mm_set_epi8(0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
                                 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                                 0x00);

  for (unsigned int i = 0; i < maxI; ++i) {
    bool end = false;
    for (unsigned int j = 0; j < maxJ;) {
      __m128i result_hi = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      __m128i result_lo = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      // Enter convolution with kernel.
      for (unsigned int x = 0; x < X; ++x) {
        for (unsigned int y = 0; y < Y; ++y) {
          const char m = kernel.at<char>(y, x);
          if (m == 0)
            continue;
          __m128i mult = _mm_set_epi16(m, m, m, m, m, m, m, m);
          unsigned char* p = (src.data + (stride * (i + y)) + x + j);
          __m128i i0 = _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
          __m128i i0_hi = _mm_and_si128(i0, mask_hi);
          __m128i i0_lo = _mm_srli_si128(_mm_and_si128(i0, mask_lo), 1);

          __m128i i_hi = _mm_mullo_epi16(i0_hi, mult);
          __m128i i_lo = _mm_mullo_epi16(i0_lo, mult);
          result_hi = _mm_add_epi16(result_hi, i_hi);
          result_lo = _mm_add_epi16(result_lo, i_lo);
        }
      }
      // Store.
      unsigned char* p_lo = (dst.data + (2 * stride * (i + cy))) + 2 * cx + 2 * j;
      unsigned char* p_hi = (dst.data + (2 * stride * (i + cy))) + 2 * cx + 2 * j + 16;
      _mm_storeu_si128(reinterpret_cast<__m128i *>(p_hi),
                       _mm_unpackhi_epi16(result_hi, result_lo));
      _mm_storeu_si128(reinterpret_cast<__m128i *>(p_lo),
                       _mm_unpacklo_epi16(result_hi, result_lo));

      // Take care about end.
      j += 16;
      if (j >= maxJ && !end) {
        j = stride - 2 - 16;
        end = true;
      }
    }
  }
}

// X and Y denote the size of the mask.
template<int X, int Y>
__inline__ void Filter2D(cv::Mat& src, cv::Mat& dst, cv::Mat& kernel) {  // NOLINT
  if (src.type() == CV_8U)
    Filter2D8U<X, Y>(src, dst, kernel);
  else if (src.type() == CV_16S)
    Filter2D16S<X, Y>(src, dst, kernel);
  else
    assert(0 && "Only CV_8U and CV_16S are supported src matrix types.");
}

#endif  // INTERNAL_VECTORIZED_FILTERS_INL_H_
