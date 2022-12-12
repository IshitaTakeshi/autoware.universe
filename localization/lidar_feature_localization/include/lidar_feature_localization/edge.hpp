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


#ifndef LIDAR_FEATURE_LOCALIZATION__EDGE_HPP_
#define LIDAR_FEATURE_LOCALIZATION__EDGE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <algorithm>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/pcl_utils.hpp"
#include "lidar_feature_library/random.hpp"

#include "lidar_feature_localization/degenerate.hpp"
#include "lidar_feature_localization/filter.hpp"
#include "lidar_feature_localization/jacobian.hpp"
#include "lidar_feature_localization/kdtree.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/point_to_vector.hpp"


Eigen::VectorXd Center(const Eigen::MatrixXd & X);

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CalcMeanAndCovariance(const Eigen::MatrixXd & X);

Eigen::Vector3d TripletCross(
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PrincipalComponents(const Eigen::Matrix3d & C);

Eigen::Matrix<double, 3, 7> MakeEdgeJacobianRow(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2);

Eigen::Vector3d MakeEdgeResidual(
  const Eigen::Isometry3d & transform,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2);

Eigen::MatrixXd GetXYZ(const pcl::PointCloud<pcl::PointXYZ> & map);

inline bool IsEdge(const Eigen::Vector3d & eigenvalues, const double threshold = 5.0)
{
  return eigenvalues(2) < eigenvalues(1) * threshold;
}

class Edge
{
public:
  Edge(const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map, const size_t n_neighbors);

  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> Make(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
    const Eigen::Isometry3d & point_to_map) const;

  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> NeighborMeanAndCovariance(
    const pcl::PointXYZ & query) const;

private:
  const KDTree kdtree_;
  const size_t n_neighbors_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__EDGE_HPP_
