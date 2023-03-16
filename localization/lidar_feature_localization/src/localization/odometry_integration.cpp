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

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_localization/odometry_integration.hpp"


OdometryIntegration::OdometryIntegration()
{
}

void OdometryIntegration::Add(
  const double time_second,
  const Eigen::Vector3d & velocity,
  const Eigen::Vector3d & angular_velocity)
{
  const Eigen::Quaterniond rotation = angular_integration_.Get(time_second);

  velocity_integration_.Add(time_second, rotation * velocity);
  angular_integration_.Add(time_second, angular_velocity);
}

Eigen::Isometry3d OdometryIntegration::Get(const double t) const
{
  const Eigen::Quaterniond rotation = angular_integration_.Get(t);
  const Eigen::Vector3d translation = velocity_integration_.Get(t);
  return MakeIsometry3d(rotation, translation);
}

Eigen::Isometry3d OdometryIntegration::InitialValue() const
{
  const Eigen::Quaterniond rotation = angular_integration_.InitialValue();
  const Eigen::Vector3d translation = velocity_integration_.InitialValue();
  return MakeIsometry3d(rotation, translation);
}
