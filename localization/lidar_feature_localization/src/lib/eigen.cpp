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

#include <string>
#include <vector>

#include "lidar_feature_library/eigen.hpp"


Eigen::Isometry3d MakeIsometry3d(const Eigen::Quaterniond & q, const Eigen::Vector3d & t)
{
  assert(std::abs(q.norm() - 1.0) < 1e-6);
  Eigen::Isometry3d transform;
  transform.linear() = q.toRotationMatrix();
  transform.translation() = t;
  return transform;
}

Eigen::VectorXd TransformXYZ(const Eigen::Isometry3d & transform, const Eigen::VectorXd & p0)
{
  assert(p0.size() >= 3);
  const size_t d = p0.size();
  Eigen::VectorXd p1(d);
  Eigen::Vector3d head = p0.head(3);
  p1.head(3) = transform * head;
  p1.tail(d - 3) = p0.tail(d - 3);
  return p1;
}

Eigen::VectorXd VectorToEigen(const std::vector<double> & values)
{
  Eigen::VectorXd v(values.size());
  for (unsigned int i = 0; i < values.size(); i++) {
    v(i) = values[i];
  }
  return v;
}

std::string EigenToString(const Eigen::MatrixXd & matrix)
{
  std::stringstream ss;
  ss << matrix;
  return ss.str();
}

Eigen::MatrixXd GetRows(
  const Eigen::MatrixXd & matrix,
  const std::vector<std::uint64_t> & indices)
{
  Eigen::MatrixXd A(indices.size(), matrix.cols());
  for (const auto & [i, index] : ranges::views::enumerate(indices)) {
    A.row(i) = matrix.row(index);
  }
  return A;
}

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> EigenDecomposition(const Eigen::Matrix3d & C)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  return solver.computeDirect(C);
}
