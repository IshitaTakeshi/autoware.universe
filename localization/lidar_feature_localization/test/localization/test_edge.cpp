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

#include <gmock/gmock.h>

#include <vector>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/optimizer.hpp"

#include "lidar_feature_library/point_type.hpp"

#include "rotationlib/quaternion.hpp"


using testing::ElementsAre;
using testing::DoubleEq;

TEST(Edge, TripletCross)
{
  {
    const Eigen::Vector3d p0(1., 2., 3.);
    const Eigen::Vector3d p1(4., 5., 6.);
    const Eigen::Vector3d p2(7., 8., 9.);
    const Eigen::Vector3d c = TripletCross(p0, p1, p2);
    const std::vector<double> v(c.data(), c.data() + c.size());
    EXPECT_THAT(v, ElementsAre(DoubleEq(0.), DoubleEq(0.), DoubleEq(0.)));
  }

  {
    const Eigen::Vector3d p0(0., 2., 1.);
    const Eigen::Vector3d p1(2., 0., 1.);
    const Eigen::Vector3d p2(1., 3., 0.);
    const Eigen::Vector3d c = TripletCross(p0, p1, p2);
    const std::vector<double> v(c.data(), c.data() + c.size());
    EXPECT_THAT(v, ElementsAre(DoubleEq(14.), DoubleEq(2.), DoubleEq(-8.)));
  }
}

TEST(Edge, PrincipalComponents)
{
  {
    const Eigen::Matrix3d A = Eigen::Matrix3d::Random();
    const Eigen::Matrix3d C = A * A.transpose();
    const auto [eigenvalues, eigenvectors] = PrincipalComponents(C);

    EXPECT_THAT(eigenvalues(0), testing::Le(eigenvalues(1)));
    EXPECT_THAT(eigenvalues(1), testing::Le(eigenvalues(2)));

    const Eigen::Matrix3d D = eigenvalues.asDiagonal();
    const Eigen::Matrix3d V = eigenvectors;

    EXPECT_THAT((C - V * D * V.inverse()).norm(), testing::Le(1e-4));

    const double lambda = eigenvalues(2);
    const Eigen::Vector3d u = eigenvectors.col(2);
    EXPECT_THAT((C * u - lambda * u).norm(), testing::Le(1e-4));
  }

  {
    Eigen::MatrixXd X(10, 3);
    for (int i = 0; i < X.rows(); i++) {
      const double x = 0.1 * i;
      X.row(i) = Eigen::Vector3d(x, 0., 0.);
    }
    const auto [mean, covariance] = CalcMeanAndCovariance(X);
    const auto [eigenvalues, eigenvectors] = PrincipalComponents(covariance);
    EXPECT_THAT((eigenvectors.col(2) - Eigen::Vector3d(1, 0, 0)).norm(), testing::Le(1e-8));
    EXPECT_NEAR(eigenvalues(0), 0., 1e-8);
    EXPECT_NEAR(eigenvalues(1), 0., 1e-8);
    EXPECT_THAT(eigenvalues(2), testing::Gt(0.));
  }
}

TEST(Edge, IsEdge)
{
  {
    const double threshold = 10.0;
    EXPECT_TRUE(IsEdge(Eigen::Vector3d(0., 1., 10.1), threshold));
    EXPECT_FALSE(IsEdge(Eigen::Vector3d(0., 1., 10.0), threshold));
  }

  {
    const double threshold = 5.0;
    EXPECT_TRUE(IsEdge(Eigen::Vector3d(10.1, 2., 0.1), threshold));
    EXPECT_FALSE(IsEdge(Eigen::Vector3d(1.0, 5.0, 0.9), threshold));
  }
}

TEST(Edge, Center)
{
  const Eigen::Matrix<double, 5, 3> A =
    (Eigen::Matrix<double, 5, 3>() <<
    4, 5, 1,
    2, 0, 4,
    6, 2, 2,
    7, 5, 9,
    0, 7, 7).finished();
  const Eigen::Vector3d c = Center(A);
  std::vector<double> v(c.data(), c.data() + c.size());
  EXPECT_THAT(v, ElementsAre(3.8, 3.8, 4.6));
}

TEST(Edge, CalcMeanAndCovariance)
{
  const Eigen::MatrixXd X =
    (Eigen::MatrixXd(4, 3) <<
    2, 8, 9,
    3, 5, 0,
    6, 5, 5,
    5, 2, 2).finished();

  const Eigen::Matrix3d expected =
    (Eigen::Matrix3d() <<
    10., -9., -6.,
    -9., 18., 21.,
    -6., 21., 46.).finished() / 4.;

  const auto [mean, covariance] = CalcMeanAndCovariance(X);

  ASSERT_EQ(mean, Center(X));

  ASSERT_EQ(covariance.rows(), 3);
  ASSERT_EQ(covariance.cols(), 3);
  EXPECT_EQ((covariance - expected).norm(), 0.);
}

TEST(Edge, ApproximateError)
{
  auto residual = [](
    const Eigen::Vector3d & p0,
    const Eigen::Vector3d & p1,
    const Eigen::Vector3d & p2,
    const Eigen::Vector3d & theta,
    const Eigen::Vector3d & t) {
      const Eigen::Quaterniond q = rotationlib::AngleAxisToQuaternion(theta);

      Eigen::Isometry3d transform;
      transform.linear() = q.toRotationMatrix();
      transform.translation() = t;

      return MakeEdgeResidual(transform, p0, p1, p2);
    };

  const Eigen::MatrixXd X =
    (Eigen::MatrixXd(5, 3) <<
    0, 0, 0,
    1, 0, 0,
    2, 0, 0,
    3, 0, 0,
    4, 0, 0).finished();

  const Eigen::Vector3d theta0(0., 0., 0.);
  const Eigen::Vector3d t0(3, 2., 1.);

  const Eigen::Vector3d dtheta(0.1, 0.1, 0.1);
  const Eigen::Vector3d dt(0.2, -0.1, 0.5);

  const Eigen::Vector3d theta1 = theta0 + dtheta;
  const Eigen::Vector3d t1 = t0 + dt;

  const Eigen::Vector3d p0(2, 1, 0);

  const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(theta0);

  const Eigen::Vector3d center = Center(X);
  const Eigen::Vector3d principal(1, 0, 0);

  const Eigen::Vector3d p1 = center - principal;
  const Eigen::Vector3d p2 = center + principal;

  const Eigen::Matrix<double, 3, 6> J = MakeEdgeJacobianRow(q0, p0, p1, p2) * MakeM(q0);

  const Eigen::Vector3d r0 = residual(p0, p1, p2, theta0, t0);
  const Eigen::Vector3d r1 = residual(p0, p1, p2, theta1, t1);

  const Vector6d delta = (Vector6d() << dtheta, dt).finished();
  const double threshold = 0.1 * (r1 - r0).norm();
  EXPECT_THAT((r1 - (r0 + J * delta)).norm(), testing::Le(threshold));
}

TEST(Edge, Convergence)
{
  std::default_random_engine generator;
  std::normal_distribution<float> distribution(0., 0.01);

  auto normal = [&]() {
      return distribution(generator);
    };

  auto make_lines = [&](const int n) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr lines(new pcl::PointCloud<pcl::PointXYZ>());

      const double s = 0.01;
      const double offset = 10.0;
      for (int x = 0; x < n; x++) {
        lines->push_back(pcl::PointXYZ(offset + s * x, normal(), normal()));
      }

      for (int y = 0; y < n; y++) {
        lines->push_back(pcl::PointXYZ(normal(), offset + s * y, normal()));
      }

      for (int z = 0; z < n; z++) {
        lines->push_back(pcl::PointXYZ(normal(), normal(), offset + s * z));
      }

      return lines;
    };

  using OptimizerType = Optimizer<Edge, pcl::PointCloud<pcl::PointXYZ>::Ptr>;

  const int max_iter = 40;
  const double huber_k = 1.345;
  const auto map = make_lines(200);

  const Eigen::Quaterniond q_true = Eigen::Quaterniond(1.0, 0.1, 0.1, -0.1).normalized();
  const Eigen::Vector3d t_true(0.7, -0.3, 0.5);

  const Eigen::Isometry3d transform_true = MakeIsometry3d(q_true, t_true);

  const auto scan = TransformPointCloud<pcl::PointXYZ>(transform_true.inverse(), map);

  const Edge edge(map, 5);

  const Eigen::Quaterniond q_initial = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d t_initial = Eigen::Vector3d::Zero();
  const Eigen::Isometry3d initial_pose = MakeIsometry3d(q_initial, t_initial);

  const OptimizerType optimizer(edge, max_iter, huber_k);
  const OptimizationResult result = optimizer.Run(scan, initial_pose);
  const Eigen::Isometry3d transform_pred = result.pose;
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(transform_pred, scan);

  EXPECT_THAT(
    (transform_true.linear() - transform_pred.linear()).norm(),
    testing::Le(0.1));
  EXPECT_THAT(
    (transform_true.translation() - transform_pred.translation()).norm(),
    testing::Le(0.05));
}
