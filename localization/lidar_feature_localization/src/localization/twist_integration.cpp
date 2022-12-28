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

#include <stdexcept>

#include "lidar_feature_localization/twist_integration.hpp"
#include "lidar_feature_localization/interpolation.hpp"

#include "rotationlib/quaternion.hpp"


bool TwistIntegration::IsInitialized() const
{
  return qs_.Size() > 0;
}

void TwistIntegration::Update(
  const double t_sec_curr,
  const Eigen::Vector3d & omega,
  const Eigen::Vector3d & v)
{
  if (!this->IsInitialized()) {
    throw std::invalid_argument("Not initialized yet");
  }

  const double dt = t_sec_curr - t_sec_;

  const Eigen::Quaterniond q0 = std::get<1>(qs_.GetLast());
  const Eigen::Quaterniond q1 = q0 * rotationlib::AngleAxisToQuaternion(omega_ * dt);

  const Eigen::Vector3d p0 = std::get<1>(ps_.GetLast());
  const Eigen::Vector3d p1 = p0 + v_ * dt;

  qs_.Insert(t_sec_curr, q1);
  ps_.Insert(t_sec_curr, p1);

  t_sec_ = t_sec_curr;
  v_ = v;
  omega_ = omega;
}

void TwistIntegration::Init(
  const double t_sec_curr,
  const Eigen::Vector3d & omega,
  const Eigen::Vector3d & v)
{
  if (this->IsInitialized()) {
    throw std::runtime_error("TwistIntegration is already initialized");
  }

  qs_.Insert(t_sec_curr, Eigen::Quaterniond::Identity());
  ps_.Insert(t_sec_curr, Eigen::Vector3d::Zero());

  v0_ = v;
  omega0_ = omega;
  t_sec_ = t_sec_curr;
  v_ = v;
  omega_ = omega;
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d> Predict(
  const Eigen::Vector3d & p0,
  const Eigen::Quaterniond & q0,
  const Eigen::Vector3d & v0,
  const Eigen::Vector3d & omega0,
  const double dt)
{
  const Eigen::Vector3d dp = v0 * dt;
  const Eigen::Quaterniond dq = rotationlib::AngleAxisToQuaternion(omega0 * dt);
  return std::make_tuple(q0 * dq, p0 + dp);
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d>
TwistIntegration::GetEarlierThanFirst(const double t_sec_curr) const
{
  const double t = qs_.GetFirstTimestamp();

  assert(t_sec_curr < t);

  const auto [_pt, p] = ps_.GetFirst();
  const auto [_qt, q] = qs_.GetFirst();
  const double dt = t_sec_curr - t;
  return Predict(p, q, v0_, omega0_, dt);
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d>
TwistIntegration::GetLaterThanLast(const double t_sec_curr) const
{
  const double t = qs_.GetLastTimestamp();

  assert(t_sec_curr > t);

  const auto [_pt, p] = ps_.GetLast();
  const auto [_qt, q] = qs_.GetLast();
  const double dt = t_sec_curr - t;
  return Predict(p, q, v_, omega_, dt);
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d>
TwistIntegration::GetInterpolated(const double t_sec_curr) const
{
  const auto [tq0, q0] = qs_.GetPrev(t_sec_curr);
  const auto [tq1, q1] = qs_.GetNext(t_sec_curr);
  const Eigen::Quaterniond q = Interpolate(q0, q1, tq0, tq1, t_sec_curr);

  const auto [tp0, p0] = ps_.GetPrev(t_sec_curr);
  const auto [tp1, p1] = ps_.GetNext(t_sec_curr);
  const Eigen::Vector3d p = Interpolate(p0, p1, tp0, tp1, t_sec_curr);

  return std::make_tuple(q, p);
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d> TwistIntegration::Get(const double t_sec_curr) const
{
  if (t_sec_curr <= qs_.GetFirstTimestamp()) {
    return this->GetEarlierThanFirst(t_sec_curr);
  }

  if (t_sec_curr >= qs_.GetLastTimestamp()) {
    return this->GetLaterThanLast(t_sec_curr);
  }

  return this->GetInterpolated(t_sec_curr);
}
