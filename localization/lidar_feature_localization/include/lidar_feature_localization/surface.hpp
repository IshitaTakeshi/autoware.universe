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


#ifndef LIDAR_FEATURE_LOCALIZATION__SURFACE_HPP_
#define LIDAR_FEATURE_LOCALIZATION__SURFACE_HPP_

#include <memory>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/kdtree.hpp"

double SignedPointPlaneDistance(const Eigen::VectorXd & w, const Eigen::VectorXd & x);

Eigen::VectorXd SignedPointPlaneDistanceVector1d(
  const Eigen::VectorXd & w, const Eigen::VectorXd & x);

double PointPlaneDistance(const Eigen::VectorXd & w, const Eigen::VectorXd & x);

bool CheckPointsDistributeAlongPlane(
  const Eigen::MatrixXd & X, const Eigen::VectorXd & w,
  const double threshold = 0.1);

Eigen::VectorXd EstimatePlaneCoefficients(const Eigen::MatrixXd & X);

Eigen::Matrix<double, 1, 7> MakeJacobianRow(
  const Eigen::Vector3d & w,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p);

class Surface
{
public:
  Surface(const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map, const int n_neighbors);

  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> Make(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
    const Eigen::Isometry3d & point_to_map) const;

  pcl::PointCloud<pcl::PointXYZ> NearestKSearch(const pcl::PointXYZ & query) const;

private:
  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> MakeFromDownsampled(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
    const Eigen::Isometry3d & point_to_map) const;

  const KDTree kdtree_;
  const int n_neighbors_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__SURFACE_HPP_
