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

  const Eigen::Vector3d omega0(0.0, 3.0, 0.0);
  const Eigen::Vector3d omega1(1.0, 0.0, 0.0);
  const Eigen::Vector3d omega2(0.0, 0.0, 2.0);

  integration.Init(1.0, omega0);
  integration.Update(2.0, omega1);
  integration.Update(3.0, omega2);

  {
    const Eigen::Quaterniond expected = rotationlib::AngleAxisToQuaternion(omega0 * (-0.5));
    const Eigen::Quaterniond q = integration.Get(0.5);
    EXPECT_LT((expected.inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond expected = rotationlib::AngleAxisToQuaternion(omega0 * 0.5);
    const Eigen::Quaterniond q = integration.Get(1.5);
    EXPECT_LT((expected.inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(omega0 * 1.0);
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega1 * 0.5);

    const Eigen::Quaterniond q = integration.Get(2.5);
    EXPECT_LT(((q0 * q1).inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(omega0 * 1.0);
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega1 * 1.0);

    const Eigen::Quaterniond q = integration.Get(3.0);
    EXPECT_LT(((q0 * q1).inverse() * q).vec().norm(), 1e-8);
  }

  {
    const Eigen::Quaterniond q0 = rotationlib::AngleAxisToQuaternion(omega0 * 1.0);
    const Eigen::Quaterniond q1 = rotationlib::AngleAxisToQuaternion(omega1 * 1.0);
    const Eigen::Quaterniond q2 = rotationlib::AngleAxisToQuaternion(omega2 * 0.5);

    const Eigen::Quaterniond q = integration.Get(3.5);
    EXPECT_LT(((q0 * q1 * q2).inverse() * q).vec().norm(), 1e-8);
  }
}
