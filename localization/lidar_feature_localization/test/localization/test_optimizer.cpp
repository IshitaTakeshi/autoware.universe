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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tuple>
#include <vector>

#include "lidar_feature_library/random.hpp"

#include "lidar_feature_localization/alignment.hpp"
#include "lidar_feature_localization/robust.hpp"
#include "lidar_feature_localization/optimizer.hpp"

#include "lidar_feature_library/eigen.hpp"

constexpr double huber_k = 1.345;

const Eigen::Isometry3d MakeTransform(const Eigen::Quaterniond & q, const Eigen::Vector3d & t)
{
  Eigen::Isometry3d transform;
  transform.linear() = q.toRotationMatrix();
  transform.translation() = t;
  return transform;
}

TEST(Alignment, SimpleDatasetConvergenceCheck)
{
  const int max_iter = 10;
  const Eigen::Quaterniond q_true = Eigen::Quaterniond(1, -1, 1, -1).normalized();
  const Eigen::Vector3d t_true(-1, 3, 2);
  const Eigen::Isometry3d transform_true = MakeTransform(q_true, t_true);

  const Eigen::Matrix<double, 4, 3> X = (
    Eigen::Matrix<double, 4, 3>() <<
      4, -3, -4,
      -3, -2, -5,
      -4, 0, 2,
      -3, -3, 3
  ).finished();

  const Eigen::MatrixXd Y = (transform_true * X.transpose()).transpose();

  using ArgumentType = std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>;

  {
    const AlignmentProblem problem;

    const Eigen::Quaterniond initial_q = Eigen::Quaterniond(1.1, -1.1, 1.1, -1.1).normalized();
    const Eigen::Vector3d initial_t(-4, -6, 3);

    const Eigen::Isometry3d initial = MakeIsometry3d(initial_q, initial_t);

    const auto [jacobians, residuals] = problem.Make(std::make_tuple(X, Y), initial);

    const Eigen::VectorXd errors = ComputeErrors(residuals);
    const auto [normalized, scale] = NormalizeErrorScale(errors);
    const Eigen::VectorXd weights = ComputeWeights(normalized, huber_k);
    const auto [dq, dt] = CalcUpdate(initial_q, weights, jacobians, residuals);

    const Eigen::Isometry3d updated = MakeIsometry3d(initial_q * dq, initial_t + dt);

    const std::vector<Eigen::VectorXd> initial_r = MakeResidual(initial, X, Y);
    const std::vector<Eigen::VectorXd> updated_r = MakeResidual(updated, X, Y);

    double initial_error = 0.;
    double updated_error = 0.;
    for (size_t i = 0; i < initial_r.size(); i++) {
      initial_error += initial_r.at(i).squaredNorm();
      updated_error += updated_r.at(i).squaredNorm();
    }
    EXPECT_TRUE(updated_error < initial_error);
  }

  {
    // start from the true pose
    const AlignmentProblem problem;
    const Optimizer<AlignmentProblem, ArgumentType> optimizer(problem, max_iter, huber_k);

    const OptimizationResult result = optimizer.Run(std::make_tuple(X, Y), transform_true);
    const Eigen::Isometry3d transform_pred = result.pose;

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.iteration, 0);
    EXPECT_THAT(result.error, testing::Lt(1e-4));
    EXPECT_THAT(result.error_scale, testing::Lt(1e-4));

    EXPECT_THAT(
      (transform_true.linear() - transform_pred.linear()).norm(),
      testing::Le(1e-4));
    EXPECT_THAT(
      (transform_true.translation() - transform_pred.translation()).norm(),
      testing::Le(1e-4));
  }

  {
    // start from a different translation
    const AlignmentProblem problem;
    const Optimizer<AlignmentProblem, ArgumentType> optimizer(problem, max_iter, huber_k);

    const Eigen::Isometry3d initial = MakeIsometry3d(q_true, Eigen::Vector3d(2, 4, 1));
    const OptimizationResult result = optimizer.Run(std::make_tuple(X, Y), initial);
    const Eigen::Isometry3d transform_pred = result.pose;

    EXPECT_TRUE(result.success);
    EXPECT_THAT(result.iteration, testing::Lt(4));
    EXPECT_THAT(result.error, testing::Lt(1e-4));
    EXPECT_THAT(result.error_scale, testing::Lt(1e-4));

    EXPECT_THAT(
      (transform_true.linear() - transform_pred.linear()).norm(),
      testing::Le(1e-4));
    EXPECT_THAT(
      (transform_true.translation() - transform_pred.translation()).norm(),
      testing::Le(1e-4));
  }

  {
    // start from a different rotation
    const AlignmentProblem problem;
    const Optimizer<AlignmentProblem, ArgumentType> optimizer(problem, max_iter, huber_k);

    const Eigen::Quaterniond q = Eigen::Quaterniond(1.1, -1.1, 1.1, -1.1).normalized();
    const Eigen::Isometry3d initial = MakeIsometry3d(q, t_true);

    const OptimizationResult result = optimizer.Run(std::make_tuple(X, Y), initial);
    const Eigen::Isometry3d transform_pred = result.pose;

    EXPECT_TRUE(result.success);
    EXPECT_THAT(result.iteration, testing::Lt(6));
    EXPECT_THAT(result.error, testing::Lt(1e-4));
    EXPECT_THAT(result.error_scale, testing::Lt(1e-4));

    EXPECT_THAT(
      (transform_true.linear() - transform_pred.linear()).norm(),
      testing::Le(1e-4));
    EXPECT_THAT(
      (transform_true.translation() - transform_pred.translation()).norm(),
      testing::Le(1e-4));
  }

  {
    const AlignmentProblem problem;
    const Optimizer<AlignmentProblem, ArgumentType> optimizer(problem, max_iter, huber_k);

    const Eigen::Quaterniond q = Eigen::Quaterniond(1.1, -1.1, 1.1, -1.1).normalized();
    const Eigen::Isometry3d initial = MakeIsometry3d(q, Eigen::Vector3d(-4, -6, 3));

    const OptimizationResult result = optimizer.Run(std::make_tuple(X, Y), initial);
    const Eigen::Isometry3d transform_pred = result.pose;

    EXPECT_TRUE(result.success);
    EXPECT_THAT(result.iteration, testing::Lt(6));
    EXPECT_THAT(result.error, testing::Lt(1e-4));
    EXPECT_THAT(result.error_scale, testing::Lt(1e-4));

    EXPECT_THAT(
      (transform_true.linear() - transform_pred.linear()).norm(),
      testing::Le(1e-4));
    EXPECT_THAT(
      (transform_true.translation() - transform_pred.translation()).norm(),
      testing::Le(1e-4));
  }
}

TEST(Alignment, ShouldReturnFalseForEmptyData)
{
  const int max_iter = 10;
  const Eigen::Matrix<double, 0, 3> X;
  const Eigen::Matrix<double, 0, 3> Y;

  using ArgumentType = std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>;

  const AlignmentProblem problem;
  const Optimizer<AlignmentProblem, ArgumentType> optimizer(problem, max_iter, huber_k);

  const OptimizationResult result = optimizer.Run(
    std::make_tuple(X, Y), Eigen::Isometry3d::Identity());

  EXPECT_EQ(result.iteration, 0);
  EXPECT_FALSE(result.success);
  EXPECT_THAT(result.error, 0.);
  EXPECT_THAT(result.error_scale, 0.);
}

TEST(Alignment, ShouldReturnFalseWhenNoConvergence)
{
  const int n = 40;
  const int d = 3;

  NormalDistribution<double> distribution_x(0.0, 1.0);
  NormalDistribution<double> distribution_y(500.0, 1000.0);

  Eigen::MatrixXd X(n, d);
  Eigen::MatrixXd Y(n, d);

  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < d; j++) {
      X(i, j) = distribution_x();
      Y(i, j) = distribution_y();
    }
  }

  using ArgumentType = std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>;
  const int max_iter = 1;

  const AlignmentProblem problem;
  const Optimizer<AlignmentProblem, ArgumentType> optimizer(problem, 1, huber_k);

  const OptimizationResult result = optimizer.Run(
    std::make_tuple(X, Y), Eigen::Isometry3d::Identity());

  EXPECT_EQ(result.iteration, max_iter);
  EXPECT_FALSE(result.success);
  EXPECT_THAT(result.error, testing::Gt(10.0));
  EXPECT_THAT(result.error_scale, testing::Gt(10.0));
}

TEST(Optimizer, MakeM)
{
  {
    const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    const Eigen::Matrix<double, 7, 6> M = MakeM(q);
    Eigen::Matrix<double, 7, 6> expected;
    expected <<
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_THAT((M - expected).norm(), testing::Le(1e-8));
  }
  {
    const Eigen::Quaterniond q(2, 4, 6, 8);
    const Eigen::Matrix<double, 7, 6> M = MakeM(q);
    Eigen::Matrix<double, 7, 6> expected;
    expected <<
      -2, -3, -4, 0, 0, 0,
      1, -4, 3, 0, 0, 0,
      4, 1, -2, 0, 0, 0,
      -3, 2, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;
    EXPECT_THAT((M - expected).norm(), testing::Le(1e-8));
  }
}

TEST(WeightedUpdate, SmokeTest)
{
  NormalDistribution<double> normal(0.0, 0.5);

  Eigen::MatrixXd J0(3, 7);
  Eigen::MatrixXd J1(3, 7);

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 7; j++) {
      J0(i, j) = normal();
      J1(i, j) = normal();
    }
  }

  std::vector<Eigen::MatrixXd> jacobians{J0, J1};

  Eigen::Vector3d r0;
  Eigen::Vector3d r1;
  for (size_t i = 0; i < 3; i++) {
    r0(i) = normal();
    r1(i) = normal();
  }

  std::vector<Eigen::VectorXd> residuals{r0, r1};

  Eigen::Vector2d weights(1., 1.);

  const Eigen::Matrix<double, 7, 6> M = MakeM(Eigen::Quaterniond::Identity());
  const Vector6d delta = WeightedUpdate(M, weights, jacobians, residuals);

  // we just make sure that delta is non zero
  // the convergence test is done in another part
  EXPECT_LT(delta.norm(), 0.5);
}

TEST(WeightedUpdate, ShouldReturnZeroIfDegenerate)
{
  const Eigen::Matrix<double, 7, 6> M = MakeM(Eigen::Quaterniond::Identity());

  Eigen::MatrixXd J0 = Eigen::MatrixXd::Zero(3, 7);
  Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(3, 7);
  std::vector<Eigen::MatrixXd> jacobians{J0, J1};

  Eigen::Vector3d r0(0.1, 0.2, 0.3);
  Eigen::Vector3d r1(0.2, 0.4, 0.5);
  std::vector<Eigen::VectorXd> residuals{r0, r1};

  Eigen::Vector2d weights(0.5, 0.5);
  const Vector6d delta = WeightedUpdate(M, weights, jacobians, residuals);
  EXPECT_EQ(delta.norm(), 0.);
}

TEST(ComputeErrors, SmokeTest)
{
  Eigen::Vector3d r0(0., 1., 2.);
  Eigen::Vector3d r1(2., 1., 1.);
  const Eigen::Vector2d errors = ComputeErrors(std::vector<Eigen::VectorXd>{r0, r1});
  Eigen::Vector2d expected(5, 6);
  EXPECT_EQ((errors - expected).norm(), 0.);
}

TEST(NormalizeErrorScale, StatisticalTest)
{
  const int N = 100000;

  const double stddev = 0.5;

  NormalDistribution<double> normal(0., stddev);

  Eigen::VectorXd errors(N);
  for (size_t i = 0; i < N; i++) {
    errors(i) = normal();
  }

  const auto [normalized, scale] = NormalizeErrorScale(errors);
  EXPECT_NEAR(SampleStandardDeviation(normalized), 1.0, 1e-2);
  EXPECT_NEAR(scale, stddev, 1e-2);
}
