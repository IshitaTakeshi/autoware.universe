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

#include "lidar_feature_localization/angular_integration.hpp"

#include "rotationlib/quaternion.hpp"


TEST(AngularIntegration, AngularIntegration)
{
  AngularIntegration integration;

  const Eigen::Vector3d omega0(0.00, 0.15, 0.00);
  const Eigen::Vector3d omega1(0.20, 0.00, 0.00);
  const Eigen::Vector3d omega2(0.00, 0.00, 0.10);

  integration.Add(10.0, omega0);
  integration.Add(20.0, omega1);
  integration.Add(30.0, omega2);

  {
    const Eigen::Quaterniond expected = rotationlib::AngleAxisToQuaternion(omega0 * (-5.0));
    const Eigen::Quaterniond q = integration.Get(5.0);
    EXPECT_LT((expected.inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega0 * 5.0);

    const Eigen::Quaterniond q = integration.Get(15.0);
    EXPECT_LT(((q0 * q1).inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(omega0 * 10.0);
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega1 * 5.0);

    const Eigen::Quaterniond q = integration.Get(25.0);
    EXPECT_LT(((q0 * q1).inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(omega0 * 10.0);
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega1 * 10.0);

    const Eigen::Quaterniond q = integration.Get(30.0);
    EXPECT_LT(((q0 * q1).inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(omega0 * 10.0);
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega1 * 10.0);
    const Eigen::Quaterniond q2 = rotationlib::AngleAxisToQuaternion(omega2 * 5.0);

    const Eigen::Quaterniond q = integration.Get(35.0);
    EXPECT_LT(((q0 * q1 * q2).inverse() * q).vec().norm(), 1e-8);
  }
}

TEST(AngularIntegration, GetReturnsInitialValueIfEmpty)
{
  AngularIntegration integration;

  const Eigen::Quaterniond q = integration.Get(10.0);
  const Eigen::Quaterniond r = integration.InitialValue();
  EXPECT_EQ((q.inverse() * r).vec().norm(), 0.0);
}
