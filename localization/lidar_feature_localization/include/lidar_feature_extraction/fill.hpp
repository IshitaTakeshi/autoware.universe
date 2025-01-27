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


#ifndef LIDAR_FEATURE_EXTRACTION__FILL_HPP_
#define LIDAR_FEATURE_EXTRACTION__FILL_HPP_

#include <algorithm>

#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/point_label.hpp"
#include "lidar_feature_extraction/range_message.hpp"


template<typename Container>
void FillFromLeft(
  Container & labels,
  const int begin_index,
  const int end_index,
  const PointLabel & label)
{
  assert(end_index <= static_cast<int>(labels.size()));
  assert(begin_index >= 0);

  for (int i = begin_index; i < end_index; i++) {
    labels.at(i) = label;
  }
}

template<typename Container>
void FillFromRight(
  Container & labels,
  const int begin_index,
  const int end_index,
  const PointLabel & label)
{
  assert(end_index <= static_cast<int>(labels.size()));
  assert(begin_index >= -1);

  for (int i = end_index; i > begin_index; i--) {
    labels.at(i) = label;
  }
}

template<typename Container>
void FillNeighbors(
  Container & labels,
  const int index,
  const int padding,
  const PointLabel & label)
{
  const int label_size = static_cast<int>(labels.size());

  const int min = std::max(-1, index - padding - 1);
  const int max = std::min(index + 1 + padding, label_size);

  FillFromRight(labels, min, index, label);
  FillFromLeft(labels, index, max, label);
}

#endif  // LIDAR_FEATURE_EXTRACTION__FILL_HPP_
