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

#include <tuple>
#include <vector>

#include "lidar_feature_localization/degenerate.hpp"
#include "lidar_feature_localization/optimizer.hpp"

#include "rotationlib/quaternion.hpp"


bool CheckConvergence(const Eigen::Quaterniond & dq, const Eigen::Vector3d & dt)
{
  return dq.vec().norm() < 1e-3 && dt.norm() < 1e-3;
}

Vector6d WeightedUpdate(
  const Eigen::Matrix<double, 7, 6> & M,
  const Eigen::VectorXd & weights,
  const std::vector<Eigen::MatrixXd> & jacobians,
  const std::vector<Eigen::VectorXd> & residuals)
{
  assert(static_cast<size_t>(weights.size()) == jacobians.size());
  assert(static_cast<size_t>(weights.size()) == residuals.size());

  // It's not so beautiful to compute these many matrices at the same time but
  // we need to avoid recomputing matrix multiplications
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(7, 7);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(7, 7);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(7);

  for (size_t i = 0; i < jacobians.size(); i++) {
    assert(jacobians.at(i).cols() == 7);

    const Eigen::MatrixXd J = jacobians.at(i);
    const Eigen::VectorXd r = residuals.at(i);
    const Eigen::MatrixXd JtJ = J.transpose() * J;

    D += JtJ;
    A += weights(i) * JtJ;
    b += weights(i) * J.transpose() * r;
  }

  if (IsDegenerate(D)) {
    return Vector6d::Zero();  // corresponds to identity
  }

  return -(M.transpose() * A * M).llt().solve(M.transpose() * b);
}

Eigen::Matrix<double, 7, 6> MakeM(const Eigen::Quaterniond & q)
{
  const Eigen::Matrix4d L = rotationlib::LeftMultiplicationMatrix(q);
  const Eigen::Matrix<double, 4, 3> Q = 0.5 * L.block<4, 3>(0, 1);

  Eigen::Matrix<double, 7, 6> M;
  M.block<4, 3>(0, 0) = Q;
  M.block<4, 3>(0, 3) = Eigen::MatrixXd::Zero(4, 3);
  M.block<3, 3>(4, 0) = Eigen::Matrix3d::Zero();
  M.block<3, 3>(4, 3) = Eigen::Matrix3d::Identity();
  return M;
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d> CalcUpdate(
  const Eigen::Quaterniond & q,
  const Eigen::VectorXd & weights,
  const std::vector<Eigen::MatrixXd> & jacobians,
  const std::vector<Eigen::VectorXd> & residuals)
{
  const Eigen::Matrix<double, 7, 6> M = MakeM(q);
  const Vector6d dx = WeightedUpdate(M, weights, jacobians, residuals);
  const Eigen::Quaterniond dq = rotationlib::AngleAxisToQuaternion(dx.head(3));
  const Eigen::Vector3d dt = dx.tail(3);
  return std::make_tuple(dq, dt);
}

Eigen::VectorXd ComputeErrors(const std::vector<Eigen::VectorXd> & residuals)
{
  Eigen::VectorXd errors(residuals.size());
  for (size_t i = 0; i < residuals.size(); i++) {
    const Eigen::VectorXd r = residuals.at(i);
    errors(i) = r.dot(r);
  }
  return errors;
}

std::tuple<Eigen::VectorXd, double> NormalizeErrorScale(const Eigen::VectorXd & errors)
{
  const double scale = Scale(errors);
  Eigen::VectorXd normalized(errors.size());
  for (int32_t i = 0; i < errors.size(); i++) {
    normalized(i) = errors(i) / (scale + 1e-16);
  }
  return std::make_tuple(normalized, scale);
}

Eigen::VectorXd ComputeWeights(
  const Eigen::VectorXd & scale_normalized_errors,
  const double huber_k)
{
  // The stddev scale_normalized_errors should be close to 1.
  Eigen::VectorXd weights(scale_normalized_errors.size());
  for (int32_t i = 0; i < scale_normalized_errors.size(); i++) {
    weights(i) = HuberDerivative(scale_normalized_errors(i), huber_k);
  }
  return weights;
}
