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

#ifndef LIDAR_FEATURE_LOCALIZATION__INTEGRATION_HPP_
#define LIDAR_FEATURE_LOCALIZATION__INTEGRATION_HPP_

#include "lidar_feature_localization/interpolation.hpp"
#include "lidar_feature_localization/stamp_sorted_objects.hpp"

inline double CalcDt(double t1, const double t0)
{
  return t1 - t0;
}

template<typename IntegratedValueT, typename InputValueT>
class Integration
{
public:
  Integration()
  {
  }

  virtual IntegratedValueT Predict(
    const IntegratedValueT & q0,
    const InputValueT & omega0,
    const double dt) const = 0;

  virtual IntegratedValueT InitialValue() const = 0;

  void Add(
    const double t_sec_curr,
    const InputValueT & omega)
  {
    if (!this->IsInitialized()) {
      this->Init(t_sec_curr, omega);
      return;
    }
    this->Update(t_sec_curr, omega);
  }

  bool IsInitialized() const
  {
    return qs_.Size() > 0;
  }

  IntegratedValueT Get(const double t_sec_curr) const
  {
    // TODO throw runtime_error if not sufficient values found in qs_

    if (t_sec_curr <= qs_.GetFirstTimestamp()) {
      return this->GetEarlierThanFirst(t_sec_curr);
    }

    if (t_sec_curr >= qs_.GetLastTimestamp()) {
      return this->GetLaterThanLast(t_sec_curr);
    }

    return this->GetInterpolated(t_sec_curr);
  }

private:
  void Update(
    const double t_sec_curr,
    const InputValueT & omega)
  {
    const double dt = CalcDt(t_sec_curr, t_sec_);
    const IntegratedValueT q0 = std::get<1>(qs_.GetLast());
    const IntegratedValueT q1 = this->Predict(q0, omega_, dt);

    qs_.Insert(t_sec_curr, q1);

    t_sec_ = t_sec_curr;
    omega_ = omega;
  }

  void Init(
    const double t_sec_curr,
    const InputValueT & omega)
  {
    qs_.Insert(t_sec_curr, this->InitialValue());

    omega0_ = omega;
    omega_ = omega;
    t_sec_ = t_sec_curr;
  }

  IntegratedValueT GetEarlierThanFirst(const double t_sec_curr) const
  {
    const double t = qs_.GetFirstTimestamp();

    assert(t_sec_curr < t);

    const auto [_, q] = qs_.GetFirst();
    const double dt = CalcDt(t_sec_curr, t);
    return this->Predict(q, omega0_, dt);
  }

  IntegratedValueT GetLaterThanLast(const double t_sec_curr) const
  {
    const double t = qs_.GetLastTimestamp();

    assert(t_sec_curr > t);

    const auto [_, q] = qs_.GetLast();
    const double dt = CalcDt(t_sec_curr, t);
    return this->Predict(q, omega_, dt);
  }

  IntegratedValueT GetInterpolated(const double t_sec_curr) const
  {
    const auto [tq0, q0] = qs_.GetPrev(t_sec_curr);
    const auto [tq1, q1] = qs_.GetNext(t_sec_curr);
    return Interpolate(q0, q1, tq0, tq1, t_sec_curr);
  }

  StampSortedObjects<IntegratedValueT> qs_;
  double t_sec_;
  InputValueT omega0_;
  InputValueT omega_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__INTEGRATION_HPP_
