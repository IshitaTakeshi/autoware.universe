// Copyright 2022 The Autoware Contributors, Takeshi Ishita
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

#include "lidar_feature_localization/debug.hpp"

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/surface.hpp"


double ComputeX(const Eigen::Vector3d & w, const double & y, const double & z)
{
  return -(1.0 / w(0)) * (w(1) * y + w(2) * z + 1);
}

double ComputeY(const Eigen::Vector3d & w, const double & z, const double & x)
{
  return -(1.0 / w(1)) * (w(2) * z + w(0) * x + 1);
}

double ComputeZ(const Eigen::Vector3d & w, const double & x, const double & y)
{
  return -(1.0 / w(2)) * (w(0) * x + w(1) * y + 1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneYZ(
  const Eigen::Vector3d & w, const Eigen::Vector3d & mu, const int n)
{
  const double scale = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (int iy = n; iy <= n; iy++) {
    for (int iz = n; iz <= n; iz++) {
      const double y = mu(1) + static_cast<double>(iy) * scale;
      const double z = mu(2) + static_cast<double>(iz) * scale;
      const double x = ComputeX(w, y, z);
      plane->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneZX(
  const Eigen::Vector3d & w, const Eigen::Vector3d & mu, const int n)
{
  const double scale = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (int iz = n; iz <= n; iz++) {
    for (int ix = n; ix <= n; ix++) {
      const double z = mu(2) + static_cast<double>(iz) * scale;
      const double x = mu(0) + static_cast<double>(ix) * scale;
      const double y = ComputeY(w, z, x);
      plane->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneXY(
  const Eigen::Vector3d & w, const Eigen::Vector3d & mu, const int n)
{
  const double scale = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (int ix = n; ix <= n; ix++) {
    for (int iy = n; iy <= n; iy++) {
      const double x = mu(0) + static_cast<double>(ix) * scale;
      const double y = mu(1) + static_cast<double>(iy) * scale;
      const double z = ComputeZ(w, x, y);
      plane->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GeneratePlane(const Eigen::MatrixXd & X)
{
  const int n = 10;
  const Eigen::Vector3d w = EstimatePlaneCoefficients(X);
  const Eigen::Vector3d mu = X.colwise().mean();

  if (!CheckPointsDistributeAlongPlane(X, w)) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
    return empty;
  }

  if (w(0) >= w(1) && w(0) >= w(2)) {
    return PlaneYZ(w, mu, n);
  }

  if (w(1) >= w(2) && w(1) >= w(0)) {
    return PlaneZX(w, mu, n);
  }

  if (w(2) >= w(0) && w(2) >= w(1)) {
    return PlaneXY(w, mu, n);
  }

  assert(false);
  return nullptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetSurfaceCloud(
  const std::shared_ptr<Surface> & surface,
  const Eigen::Isometry3d & point_to_map,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan)
{
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(point_to_map, surface_scan);

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (const pcl::PointXYZ & query : *transformed) {
    const pcl::PointCloud<pcl::PointXYZ> neighbors = surface->NearestKSearch(query);
    const Eigen::MatrixXd X = GetXYZ(neighbors);
    *plane += *GeneratePlane(X);
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetEdgeCloud(
  const std::shared_ptr<Edge> & edge,
  const Eigen::Isometry3d & point_to_map,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan)
{
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(point_to_map, edge_scan);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const pcl::PointXYZ & query : *transformed) {
    const auto [mean, covariance] = edge->NeighborMeanAndCovariance(query);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ed = EigenDecomposition(covariance);
    const Eigen::Vector3d eigenvalues = ed.eigenvalues();

    if (!IsEdge(eigenvalues)) {
      continue;
    }

    const Eigen::Vector3d principal = ed.eigenvectors().col(2);

    for (int i = -20; i <= 20; i++) {
      const Eigen::Vector3d v = mean + 0.1 * static_cast<double>(i) * principal;
      const pcl::PointXYZ p = PointXYZVectorConversion::VectorToPoint(v);
      cloud->push_back(p);
    }
  }
  return cloud;
}
