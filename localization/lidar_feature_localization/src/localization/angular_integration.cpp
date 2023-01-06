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

#include "lidar_feature_localization/angular_integration.hpp"
#include "lidar_feature_localization/interpolation.hpp"

#include "rotationlib/quaternion.hpp"


bool AngularIntegration::IsInitialized() const
{
  return qs_.Size() > 0;
}

void AngularIntegration::Update(
  const double t_sec_curr,
  const Eigen::Vector3d & omega)
{
  if (!this->IsInitialized()) {
    throw std::invalid_argument("Not initialized yet");
  }

  const double dt = t_sec_curr - t_sec_;

  const Eigen::Quaterniond q0 = std::get<1>(qs_.GetLast());
  const Eigen::Quaterniond q1 = q0 * rotationlib::AngleAxisToQuaternion(omega_ * dt);

  qs_.Insert(t_sec_curr, q1);

  t_sec_ = t_sec_curr;
  omega_ = omega;
}

void AngularIntegration::Init(
  const double t_sec_curr,
  const Eigen::Vector3d & omega)
{
  if (this->IsInitialized()) {
    throw std::runtime_error("AngularIntegration is already initialized");
  }

  qs_.Insert(t_sec_curr, Eigen::Quaterniond::Identity());

  omega0_ = omega;
  omega_ = omega;
  t_sec_ = t_sec_curr;
}

Eigen::Quaterniond Predict(
  const Eigen::Quaterniond & q0,
  const Eigen::Vector3d & omega0,
  const double dt)
{
  return q0 * rotationlib::AngleAxisToQuaternion(omega0 * dt);
}

Eigen::Quaterniond AngularIntegration::GetEarlierThanFirst(const double t_sec_curr) const
{
  const double t = qs_.GetFirstTimestamp();

  assert(t_sec_curr < t);

  const auto [_qt, q] = qs_.GetFirst();
  const double dt = t_sec_curr - t;
  return Predict(q, omega0_, dt);
}

Eigen::Quaterniond AngularIntegration::GetLaterThanLast(const double t_sec_curr) const
{
  const double t = qs_.GetLastTimestamp();

  assert(t_sec_curr > t);

  const auto [_qt, q] = qs_.GetLast();
  const double dt = t_sec_curr - t;
  return Predict(q, omega_, dt);
}

Eigen::Quaterniond AngularIntegration::GetInterpolated(const double t_sec_curr) const
{
  const auto [tq0, q0] = qs_.GetPrev(t_sec_curr);
  const auto [tq1, q1] = qs_.GetNext(t_sec_curr);
  return Interpolate(q0, q1, tq0, tq1, t_sec_curr);
}

Eigen::Quaterniond AngularIntegration::Get(const double t_sec_curr) const
{
  if (t_sec_curr <= qs_.GetFirstTimestamp()) {
    return this->GetEarlierThanFirst(t_sec_curr);
  }

  if (t_sec_curr >= qs_.GetLastTimestamp()) {
    return this->GetLaterThanLast(t_sec_curr);
  }

  return this->GetInterpolated(t_sec_curr);
}
