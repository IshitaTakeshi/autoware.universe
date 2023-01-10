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
//    * Neither the name of the Autoware Contributors, Takeshi Ishita nor the names of its
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

#include <gtest/gtest.h>
#include <cmath>

#include "lidar_feature_localization/odometry_integration.hpp"


inline double PoseDiffVecNorm(const Eigen::Quaterniond & qa, const Eigen::Quaterniond & qb)
{
  return (qa.inverse() * qb).vec().norm();
}

TEST(OdometryIntegration, OdometryIntegration)
{
  const Eigen::Vector3d omega(0., 0., 2.0 * M_PI);
  const Eigen::Vector3d velocity(2.0 * M_PI, 0., 0.);

  OdometryIntegration integration;

  for (size_t i = 0; i < 3600; i++) {
    const double t = i / 3600.;
    integration.Add(t, velocity, omega);
  }

  {
    const double t = 900. / 3600.;
    const Eigen::Isometry3d result = integration.Get(t);
    const Eigen::Quaterniond q(result.rotation());

    const Eigen::Vector3d expected_translation(1., 1., 0.);
    EXPECT_LT((result.translation() - expected_translation).norm(), 0.01);

    const Eigen::Quaterniond expected_rotation = rotationlib::AngleAxisToQuaternion(omega * t);
    const Eigen::Quaterniond result_rotation(result.rotation());
    EXPECT_LT(PoseDiffVecNorm(result_rotation, expected_rotation), 0.01);
  }

  {
    const double t = 2700. / 3600.;
    const Eigen::Isometry3d result = integration.Get(t);
    const Eigen::Quaterniond q(result.rotation());

    const Eigen::Vector3d expected_translation(-1., 1., 0.);
    EXPECT_LT((result.translation() - expected_translation).norm(), 0.01);

    const Eigen::Quaterniond expected_rotation = rotationlib::AngleAxisToQuaternion(omega * t);
    const Eigen::Quaterniond result_rotation(result.rotation());
    EXPECT_LT(PoseDiffVecNorm(result_rotation, expected_rotation), 0.01);
  }
}
