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

#ifndef LIDAR_FEATURE_LOCALIZATION__ANGULAR_INTEGRATION_HPP_
#define LIDAR_FEATURE_LOCALIZATION__ANGULAR_INTEGRATION_HPP_

#include <tuple>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "lidar_feature_localization/integration.hpp"

#include "rotationlib/quaternion.hpp"


class AngularIntegration : public Integration<Eigen::Quaterniond, Eigen::Vector3d>
{
public:
  Eigen::Quaterniond Predict(
    const Eigen::Quaterniond & q0,
    const Eigen::Vector3d & omega0,
    const double dt) const
  {
    return q0 * rotationlib::AngleAxisToQuaternion(omega0 * dt);
  }

  Eigen::Quaterniond InitialValue() const
  {
    return Eigen::Quaterniond::Identity();
  }
};

#endif  // LIDAR_FEATURE_LOCALIZATION__ANGULAR_INTEGRATION_HPP_
