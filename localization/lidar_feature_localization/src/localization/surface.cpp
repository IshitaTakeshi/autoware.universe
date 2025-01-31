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


#include <memory>
#include <tuple>
#include <vector>

#include "lidar_feature_library/downsample.hpp"
#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/pcl_utils.hpp"

#include "lidar_feature_localization/eigen.hpp"
#include "lidar_feature_localization/filter.hpp"
#include "lidar_feature_localization/jacobian.hpp"
#include "lidar_feature_localization/kdtree.hpp"
#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/point_to_vector.hpp"
#include "lidar_feature_localization/surface.hpp"

const double plane_bias = 1.0;

double SignedPointPlaneDistance(const Eigen::VectorXd & w, const Eigen::VectorXd & x)
{
  assert(w.size() == x.size());
  return (w.dot(x) + plane_bias) / w.norm();
}

Eigen::VectorXd SignedPointPlaneDistanceVector1d(
  const Eigen::VectorXd & w, const Eigen::VectorXd & x)
{
  Eigen::VectorXd d(1);
  d(0) = SignedPointPlaneDistance(w, x);
  return d;
}

double PointPlaneDistance(const Eigen::VectorXd & w, const Eigen::VectorXd & x)
{
  return std::abs(SignedPointPlaneDistance(w, x));
}

bool CheckPointsDistributeAlongPlane(
  const Eigen::MatrixXd & X, const Eigen::VectorXd & w, const double threshold)
{
  for (int j = 0; j < X.rows(); j++) {
    const Eigen::VectorXd x = X.row(j);
    if (PointPlaneDistance(w, x) > threshold) {
      return false;
    }
  }
  return true;
}

Eigen::VectorXd EstimatePlaneCoefficients(const Eigen::MatrixXd & X)
{
  const Eigen::VectorXd g = Eigen::VectorXd::Constant(X.rows(), -plane_bias);
  return SolveLinear(X, g);
}

Eigen::Matrix<double, 1, 7> MakeJacobianRow(
  const Eigen::Vector3d & w,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p)
{
  const Eigen::Matrix<double, 3, 4> drpdq = rotationlib::DRpDq(q, p);
  const Eigen::Vector3d u = w / w.norm();
  return (Eigen::Matrix<double, 1, 7>() << u.transpose() * drpdq, u.transpose()).finished();
}

Surface::Surface(const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map, const int n_neighbors)
: kdtree_(surface_map), n_neighbors_(n_neighbors)
{
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> Surface::Make(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
  const Eigen::Isometry3d & point_to_map) const
{
  // TODO(IshitaTakeshi) Make the leaf size specifiable
  const auto downsampled = Downsample<pcl::PointXYZ>(scan, 1.0);
  return this->MakeFromDownsampled(downsampled, point_to_map);
}

pcl::PointCloud<pcl::PointXYZ> Surface::NearestKSearch(const pcl::PointXYZ & query) const
{
  return kdtree_.NearestKSearch(query, n_neighbors_);
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> Surface::MakeFromDownsampled(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
  const Eigen::Isometry3d & point_to_map) const
{
  const size_t n = scan->size();

  const Eigen::Quaterniond q(point_to_map.rotation());
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(point_to_map, scan);

  std::vector<Eigen::MatrixXd> jacobians;
  std::vector<Eigen::VectorXd> residuals;
  for (size_t i = 0; i < n; i++) {
    const pcl::PointXYZ query = transformed->at(i);

    const pcl::PointCloud<pcl::PointXYZ> neighbors = this->NearestKSearch(query);

    const Eigen::MatrixXd X = GetXYZ(neighbors);

    const Eigen::Vector3d w = EstimatePlaneCoefficients(X);

    if (!CheckPointsDistributeAlongPlane(X, w)) {
      continue;
    }

    const Eigen::Vector3d p = PointXYZVectorConversion::PointToVector(scan->at(i));
    const Eigen::Vector3d g = PointXYZVectorConversion::PointToVector(query);

    jacobians.push_back(MakeJacobianRow(w, q, p));
    residuals.push_back(SignedPointPlaneDistanceVector1d(w, g));
  }

  return std::make_tuple(jacobians, residuals);
}
