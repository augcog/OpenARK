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

#ifndef BRISK_BRISK_DESCRIPTOR_EXTRACTOR_H_
#define BRISK_BRISK_DESCRIPTOR_EXTRACTOR_H_

#include <bitset>
#include <string>
#include <vector>
#include <agast/wrap-opencv.h>

#include <brisk/internal/helper-structures.h>
#include <brisk/internal/macros.h>

namespace brisk {
#if CV_MAJOR_VERSION < 3
class BriskDescriptorExtractor : public cv::DescriptorExtractor {
#else
class BriskDescriptorExtractor : public cv::Feature2D {
#endif
 public:
  friend class BriskFeature;

  enum Version {
    briskV1 = 1,
    briskV2 = 2
  };

  // Create a descriptor with standard pattern.
  BriskDescriptorExtractor(
      bool rotationInvariant = true,
      bool scaleInvariant =true,
      int version = Version::briskV2, float patternScale=1.0);
  // Create a descriptor with custom pattern file.
  BriskDescriptorExtractor(
      const std::string& fname,
      bool rotationInvariant = true,
      bool scaleInvariant = true,
      float patternScale=1.0 );
  virtual ~BriskDescriptorExtractor();

  int descriptorSize() const override;
  int descriptorType() const override;

  bool rotationInvariance;
  bool scaleInvariance;

#if CV_MAJOR_VERSION < 3
  virtual void compute(const cv::Mat& image,
                       std::vector<cv::KeyPoint>& keypoints,
                       cv::Mat& descriptors) const {
    computeImpl(image, keypoints, descriptors);
  }
#else
  virtual void compute(cv::InputArray image,
                       std::vector<cv::KeyPoint>& keypoints,
                       cv::OutputArray descriptors) override {
    cv::Mat descriptors_;
    doDescriptorComputation(image.getMat(), keypoints, descriptors_);
    descriptors.getMatRef() = descriptors_;
  }
#endif

 protected:
#if  CV_MAJOR_VERSION < 3
  void computeImpl(const cv::Mat& image,
                   std::vector<cv::KeyPoint>& keypoints,
                   cv::Mat& descriptors) const override {
    doDescriptorComputation(image, keypoints, descriptors);
  }
#endif

  void setDescriptorBits(int keypoint_idx, const int* values,
                         cv::Mat* descriptors) const;

  void AllocateDescriptors(size_t count, cv::Mat& descriptors) const;

  void doDescriptorComputation(const cv::Mat& image,
                               std::vector<cv::KeyPoint>& keypoints,
                               cv::Mat& descriptors) const;

  // Legacy BRISK 1.0
  // Call this to generate the kernel:
  // Circle of radius r (pixels), with n points;
  // Short pairings with dMax, long pairings with dMin.
  void generateKernel(std::vector<float> &radiusList,
                      std::vector<int> &numberList, float dMax = 5.85f,
                      float dMin = 8.2f, std::vector<int> indexChange =
                          std::vector<int>());

  void InitFromStream(bool rotationInvariant, bool scaleInvariant,
                      std::istream& pattern_stream, float patternScale=1.0);
  template<typename ImgPixel_T, typename IntegralPixel_T>
  __inline__ IntegralPixel_T SmoothedIntensity(const cv::Mat& image,
                                               const cv::Mat& integral,
                                               const float key_x,
                                               const float key_y,
                                               const unsigned int scale,
                                               const unsigned int rot,
                                               const unsigned int point) const;
  // Pattern properties.
  brisk::BriskPatternPoint* patternPoints_;
  // Total number of collocation points.
  unsigned int points_;
  // Lists the scaling per scale index [scale].
  float* scaleList_;
  // Lists the total pattern size per scale index [scale].
  unsigned int* sizeList_;
  // Scales discretization
  static const unsigned int scales_;
  // Span of sizes 40->4 Octaves - else, this needs to be adjusted...
  static const float scalerange_;
  // Discretization of the rotation look-up.
  static const unsigned int n_rot_;

  // Pairs.
  // Number of unsigned chars the descriptor consists of.
  int strings_;
  // Short pair maximum distance.
  float dMax_;
  // Long pair maximum distance.
  float dMin_;
  // Pairs for which d < _dMax.
  brisk::BriskShortPair* shortPairs_;
  // Pairs for which d > _dMin.
  brisk::BriskLongPair* longPairs_;
  // Number of shortPairs.
  unsigned int noShortPairs_;
  // Number of longPairs.
  unsigned int noLongPairs_;

  // General size.
  static const float basicSize_;
};
}  // namespace brisk

#endif  // BRISK_BRISK_DESCRIPTOR_EXTRACTOR_H_
