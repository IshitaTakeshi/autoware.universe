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

#ifndef LIDAR_FEATURE_LOCALIZATION__PROBLEM_HPP_
#define LIDAR_FEATURE_LOCALIZATION__PROBLEM_HPP_

#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <algorithm>
#include <memory>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/edge_surface_scan.hpp"
#include "lidar_feature_localization/degenerate.hpp"
#include "lidar_feature_localization/problem.hpp"
#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/surface.hpp"


class LOAMOptimizationProblem
{
public:
  LOAMOptimizationProblem(
    const std::shared_ptr<Edge> & edge, const std::shared_ptr<Surface> & surface);

  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>>
  Make(const EdgeSurfaceScan & edge_surface_scan, const Eigen::Isometry3d & point_to_map) const;

private:
  const std::shared_ptr<Edge> edge_;
  const std::shared_ptr<Surface> surface_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__PROBLEM_HPP_
