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

#ifndef LIDAR_FEATURE_EXTRACTION__HYPER_PARAMETER_HPP_
#define LIDAR_FEATURE_EXTRACTION__HYPER_PARAMETER_HPP_

#include "lidar_feature_library/warning.hpp"

struct HyperParameters
{
  explicit HyperParameters(rclcpp::Node * node)
  : warning_(node),
    padding(node->declare_parameter("convolution_padding", 5)),
    distance_diff_threshold(node->declare_parameter("distance_diff_threshold", 0.3)),
    parallel_beam_min_range_ratio(node->declare_parameter("parallel_beam_min_range_ratio", 0.02)),
    edge_threshold(node->declare_parameter("edge_threshold", 50.0)),
    surface_threshold(node->declare_parameter("surface_threshold", 0.05)),
    min_range(node->declare_parameter("min_range", 0.1)),
    max_range(node->declare_parameter("max_range", 1000.0)),
    n_blocks(node->declare_parameter("n_blocks", 6)),
    max_iter(node->declare_parameter("max_iter", 40)),
    n_edge_neighbors(node->declare_parameter("n_edge_neighbors", 10)),
    n_surface_neighbors(node->declare_parameter("n_surface_neighbors", 20)),
    huber_k(node->declare_parameter("huber_k", 1.345))
  {
    warning_.Info(fmt::format("convolution_padding = {}", padding));
    warning_.Info(fmt::format("distance_diff_threshold = {}", distance_diff_threshold));
    warning_.Info(fmt::format("parallel_beam_min_range_ratio = {}", parallel_beam_min_range_ratio));
    warning_.Info(fmt::format("edge_threshold = {}", edge_threshold));
    warning_.Info(fmt::format("surface_threshold = {}", surface_threshold));
    warning_.Info(fmt::format("min_range = {}", min_range));
    warning_.Info(fmt::format("max_range = {}", max_range));
    warning_.Info(fmt::format("n_blocks = {}", n_blocks));
    warning_.Info(fmt::format("max_iter = {}", max_iter));
    warning_.Info(fmt::format("n_edge_neighbors = {}", n_edge_neighbors));
    warning_.Info(fmt::format("n_surface_neighbors = {}", n_surface_neighbors));
    warning_.Info(fmt::format("huber_k = {}", huber_k));

    assert(padding > 0);
    assert(distance_diff_threshold > 0);
    assert(parallel_beam_min_range_ratio > 0);
    assert(edge_threshold > 0);
    assert(surface_threshold > 0);
    assert(min_range > 0);
    assert(max_range > 0);
    assert(n_blocks > 0);
  }

  const Warning warning_;
  const int padding;
  const double distance_diff_threshold;
  const double parallel_beam_min_range_ratio;
  const double edge_threshold;
  const double surface_threshold;
  const double min_range;
  const double max_range;
  const int n_blocks;
  const int max_iter;
  const int n_edge_neighbors;
  const int n_surface_neighbors;
  const double huber_k;
};

#endif  // LIDAR_FEATURE_EXTRACTION__HYPER_PARAMETER_HPP_
