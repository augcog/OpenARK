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

#ifndef BRISK_BRISK_FEATURE_H_
#define BRISK_BRISK_FEATURE_H_

#include <limits>
#include <vector>
#include <agast/wrap-opencv.h>

#include <brisk/brisk-descriptor-extractor.h>
#include <brisk/brisk-feature-detector.h>

#include <brisk/harris-feature-detector.h>
#include <brisk/harris-score-calculator.h>
#include <brisk/scale-space-feature-detector.h>

#ifndef __ARM_NEON__
namespace brisk {
class BriskFeature : public cv::Feature2D {
 public:
  BriskFeature(double uniformityRadius, size_t octaves, 
               double absoluteThreshold = 0,
               size_t maxNumKpt = std::numeric_limits < size_t > ::max(),
               bool rotationInvariant = true, bool scaleInvariant = true,
               int extractorVersion=BriskDescriptorExtractor::Version::briskV2)
      : _briskDetector(uniformityRadius, octaves, absoluteThreshold, maxNumKpt),
        _briskExtractor(rotationInvariant, scaleInvariant, extractorVersion) { }

  virtual ~BriskFeature() { }

  // Inherited from cv::DescriptorExtractor interface.
  int descriptorSize() const override {
    return _briskExtractor.descriptorSize();
  }
  int descriptorType() const override {
    return _briskExtractor.descriptorType();
  }

  // Inherited from cv::Feature2D interface.
#if CV_MAJOR_VERSION < 3
  void operator()(cv::InputArray image, cv::InputArray mask,
                          std::vector<cv::KeyPoint>& keypoints,
                          cv::OutputArray descriptors,
                          bool useProvidedKeypoints = false) const override {
#else
  void detectAndCompute(cv::InputArray image, cv::InputArray mask,
                          std::vector<cv::KeyPoint>& keypoints,
                          cv::OutputArray descriptors,
                          bool useProvidedKeypoints = false) override {
#endif
    if (!useProvidedKeypoints) {
      keypoints.clear();
    }

    // Convert input output arrays:
    cv::Mat descriptors_;
    cv::Mat image_ = image.getMat();
    cv::Mat mask_ = mask.getMat();

    // Run the detection. Take provided keypoints.
    _briskDetector.detect(image_, keypoints, mask_);

    // Run the extraction.
    _briskExtractor.compute(image_, keypoints, descriptors_);
    descriptors.getMatRef() = descriptors_;
  }

 protected:
#if CV_MAJOR_VERSION < 3
  // Inherited from cv::FeatureDetector interface.
  void detectImpl(const cv::Mat& image,
                          std::vector<cv::KeyPoint>& keypoints,
                          const cv::Mat& mask = cv::Mat()) const override {
    _briskDetector.detect(image, keypoints, mask);
  }
#endif
    
#if CV_MAJOR_VERSION < 3
  // Inherited from cv::DescriptorExtractor interface.
  void computeImpl(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::Mat& descriptors) const override {
    _briskExtractor.computeImpl(image, keypoints, descriptors);
  }
#endif
    

  brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator> _briskDetector;
  brisk::BriskDescriptorExtractor _briskExtractor;
};
}  // namespace brisk
#endif  // __ARM_NEON__
#endif  // BRISK_BRISK_FEATURE_H_
