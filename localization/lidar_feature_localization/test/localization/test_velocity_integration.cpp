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

#include <gmock/gmock.h>

#include "lidar_feature_localization/velocity_integration.hpp"

#include "rotationlib/quaternion.hpp"


TEST(VelocityIntegration, VelocityIntegration)
{
  VelocityIntegration integration;

  const Eigen::Vector3d v0(2.0, 0.0, 0.0);
  const Eigen::Vector3d v1(0.0, 0.5, 0.0);
  const Eigen::Vector3d v2(0.0, 0.0, 1.0);

  integration.Add(10.0, v0);
  integration.Add(20.0, v1);
  integration.Add(30.0, v2);

  {
    const Eigen::Vector3d expected = v0 * (-10.0);
    const Eigen::Vector3d p = integration.Get(0.0);
    EXPECT_EQ((p - expected).norm(), 0.);
  }

  {
    const Eigen::Vector3d expected = Eigen::Vector3d::Zero();
    const Eigen::Vector3d p = integration.Get(10.0);
    EXPECT_EQ((p - expected).norm(), 0.);
  }

  {
    const Eigen::Vector3d expected = v0 * 5.0;
    const Eigen::Vector3d p = integration.Get(15.0);
    EXPECT_EQ((p - expected).norm(), 0.);
  }

  {
    const Eigen::Vector3d expected = v0 * 10.0 + v1 * 5.0;
    const Eigen::Vector3d p = integration.Get(25.0);
    EXPECT_EQ((p - expected).norm(), 0.);
  }

  {
    const Eigen::Vector3d expected = v0 * 10.0 + v1 * 10.0;
    const Eigen::Vector3d p = integration.Get(30.0);
    EXPECT_EQ((p - expected).norm(), 0.);
  }

  {
    const Eigen::Vector3d expected = v0 * 10.0 + v1 * 10.0 + v2 * 5.0;
    const Eigen::Vector3d p = integration.Get(35.0);
    EXPECT_EQ((p - expected).norm(), 0.);
  }
}

TEST(VelocityIntegration, GetReturnsInitialValueIfEmpty)
{
  VelocityIntegration integration;

  const Eigen::Vector3d v = integration.Get(10.0);
  const Eigen::Vector3d u = integration.InitialValue();
  EXPECT_EQ((v - u).norm(), 0.0);
}
