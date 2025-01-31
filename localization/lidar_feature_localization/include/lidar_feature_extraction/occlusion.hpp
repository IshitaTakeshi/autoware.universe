// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LIDAR_FEATURE_EXTRACTION__OCCLUSION_HPP_
#define LIDAR_FEATURE_EXTRACTION__OCCLUSION_HPP_

#include <algorithm>
#include <vector>

#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/neighbor.hpp"

template<typename PointT>
void FromLeft(
  std::vector<PointLabel> & labels,
  const Range<PointT> & range,
  const int padding,
  const double distance_diff_threshold)
{
  const int size = static_cast<int>(labels.size());
  for (int i = 0; i < size - 1; i++) {
    const double range0 = range(i + 0);
    const double range1 = range(i + 1);

    if (range1 > range0 + distance_diff_threshold) {
      const int k = std::min(i + padding + 2, size);
      FillFromLeft(labels, i + 1, k, PointLabel::Occluded);
    }
  }
}

template<typename PointT>
void FromRight(
  std::vector<PointLabel> & labels,
  const Range<PointT> & range,
  const int padding,
  const double distance_diff_threshold)
{
  for (int i = labels.size() - 1; i > 0; i--) {
    const double range1 = range(i - 1);
    const double range0 = range(i - 0);

    if (range1 > range0 + distance_diff_threshold) {
      const int k = std::max(i - padding - 2, -1);
      FillFromRight(labels, k, i - 1, PointLabel::Occluded);
    }
  }
}

template<typename PointT>
void LabelOccludedPoints(
  std::vector<PointLabel> & labels,
  const Range<PointT> & range,
  const int padding,
  const double distance_diff_threshold)
{
  FromLeft(labels, range, padding, distance_diff_threshold);
  FromRight(labels, range, padding, distance_diff_threshold);
}

#endif  // LIDAR_FEATURE_EXTRACTION__OCCLUSION_HPP_
