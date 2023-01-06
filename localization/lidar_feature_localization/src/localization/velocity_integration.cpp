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

#include "lidar_feature_localization/velocity_integration.hpp"
#include "lidar_feature_localization/interpolation.hpp"


bool VelocityIntegration::IsInitialized() const
{
  return ps_.Size() > 0;
}

void VelocityIntegration::Update(
  const double t_sec_curr,
  const Eigen::Vector3d & v)
{
  if (!this->IsInitialized()) {
    throw std::invalid_argument("Not initialized yet");
  }

  const double dt = t_sec_curr - t_sec_;

  const Eigen::Vector3d p0 = std::get<1>(ps_.GetLast());
  const Eigen::Vector3d p1 = p0 + v_ * dt;

  ps_.Insert(t_sec_curr, p1);

  t_sec_ = t_sec_curr;
  v_ = v;
}

void VelocityIntegration::Init(
  const double t_sec_curr,
  const Eigen::Vector3d & v)
{
  if (this->IsInitialized()) {
    throw std::runtime_error("VelocityIntegration is already initialized");
  }

  ps_.Insert(t_sec_curr, Eigen::Vector3d::Zero());

  v0_ = v;
  v_ = v;
  t_sec_ = t_sec_curr;
}

Eigen::Vector3d Predict(
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & v0,
  const double dt)
{
  return p0 + v0 * dt;
}

Eigen::Vector3d VelocityIntegration::GetEarlierThanFirst(const double t_sec_curr) const
{
  const double t = ps_.GetFirstTimestamp();

  assert(t_sec_curr < t);

  const auto [_pt, p] = ps_.GetFirst();
  const double dt = t_sec_curr - t;
  return Predict(p, v0_, dt);
}

Eigen::Vector3d VelocityIntegration::GetLaterThanLast(const double t_sec_curr) const
{
  const double t = ps_.GetLastTimestamp();

  assert(t_sec_curr > t);

  const auto [_pt, p] = ps_.GetLast();
  const double dt = t_sec_curr - t;
  return Predict(p, v_, dt);
}

Eigen::Vector3d VelocityIntegration::GetInterpolated(const double t_sec_curr) const
{
  const auto [tp0, p0] = ps_.GetPrev(t_sec_curr);
  const auto [tp1, p1] = ps_.GetNext(t_sec_curr);
  return Interpolate(p0, p1, tp0, tp1, t_sec_curr);
}

Eigen::Vector3d VelocityIntegration::Get(const double t_sec_curr) const
{
  if (t_sec_curr <= ps_.GetFirstTimestamp()) {
    return this->GetEarlierThanFirst(t_sec_curr);
  }

  if (t_sec_curr >= ps_.GetLastTimestamp()) {
    return this->GetLaterThanLast(t_sec_curr);
  }

  return this->GetInterpolated(t_sec_curr);
}
