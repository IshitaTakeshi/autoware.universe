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

#include <Eigen/Eigenvalues>

#include <tuple>

#include "lidar_feature_library/algorithm.hpp"

#include "lidar_feature_localization/eigen.hpp"
#include "lidar_feature_localization/edge.hpp"


Eigen::VectorXd Center(const Eigen::MatrixXd & X)
{
  return X.colwise().mean();
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CalcMeanAndCovariance(const Eigen::MatrixXd & X)
{
  const Eigen::VectorXd mean = Center(X);
  const Eigen::MatrixXd D = X.rowwise() - mean.transpose();
  const Eigen::MatrixXd covariance = D.transpose() * D / X.rows();
  return std::make_tuple(mean, covariance);
}

Eigen::Vector3d TripletCross(
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  return (p2 - p1).cross((p0 - p1).cross(p0 - p2));
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PrincipalComponents(const Eigen::Matrix3d & C)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(C);
  return {solver.eigenvalues(), solver.eigenvectors()};
}

Eigen::Matrix<double, 3, 7> MakeEdgeJacobianRow(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  const Eigen::Matrix<double, 3, 4> drpdq = rotationlib::DRpDq(q, p0);
  const Eigen::Matrix3d K = rotationlib::Hat(p2 - p1);
  return (Eigen::Matrix<double, 3, 7>() << K * drpdq, K).finished();
}

Eigen::Vector3d MakeEdgeResidual(
  const Eigen::Isometry3d & transform,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  const Eigen::Vector3d p = transform * p0;
  return (p - p1).cross(p - p2);
}

Edge::Edge(const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map, const size_t n_neighbors)
: kdtree_(edge_map), n_neighbors_(n_neighbors)
{
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> Edge::Make(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
  const Eigen::Isometry3d & point_to_map) const
{
  // f(dx) \approx f(0) + J * dx + dx^T * H * dx
  // dx can be obtained by solving H * dx = -J

  const size_t n = scan->size();

  const Eigen::Quaterniond q(point_to_map.rotation());

  const auto transformed = TransformPointCloud<pcl::PointXYZ>(point_to_map, scan);

  std::vector<Eigen::MatrixXd> jacobians;
  std::vector<Eigen::VectorXd> residuals;

  for (size_t i = 0; i < n; i++) {
    const pcl::PointXYZ query = transformed->at(i);
    const auto [mean, covariance] = this->NeighborMeanAndCovariance(query);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ed = EigenDecomposition(covariance);
    const Eigen::Vector3d eigenvalues = ed.eigenvalues();

    if (IsEdge(eigenvalues)) {
      continue;
    }

    const Eigen::Vector3d principal = ed.eigenvectors().col(2);
    const Eigen::Vector3d p0 = PointXYZVectorConversion::PointToVector(scan->at(i));
    const Eigen::Vector3d p1 = mean - principal;
    const Eigen::Vector3d p2 = mean + principal;

    jacobians.push_back(MakeEdgeJacobianRow(q, p0, p1, p2));
    residuals.push_back(MakeEdgeResidual(point_to_map, p0, p1, p2));
  }

  return std::make_tuple(jacobians, residuals);
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> Edge::NeighborMeanAndCovariance(
  const pcl::PointXYZ & query) const
{
  const pcl::PointCloud<pcl::PointXYZ> neighbors = kdtree_.NearestKSearch(query, n_neighbors_);
  const Eigen::MatrixXd X = GetXYZ(neighbors);
  return CalcMeanAndCovariance(X);
}
