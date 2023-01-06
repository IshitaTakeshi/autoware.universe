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

#include "lidar_feature_localization/stamp_sorted_objects.hpp"


class AngularIntegration
{
public:
  AngularIntegration()
  : omega_(Eigen::Vector3d())
  {
  }

  bool IsInitialized() const;

  void Update(
    const double t_sec_curr,
    const Eigen::Vector3d & v);

  void Init(
    const double t_sec_curr,
    const Eigen::Vector3d & v);

  Eigen::Quaterniond Get(const double t_sec_curr) const;

private:
  Eigen::Quaterniond GetEarlierThanFirst(const double t_sec_curr) const;
  Eigen::Quaterniond GetLaterThanLast(const double t_sec_curr) const;
  Eigen::Quaterniond GetInterpolated(const double t_sec_curr) const;

  StampSortedObjects<Eigen::Quaterniond> qs_;
  double t_sec_;
  Eigen::Vector3d omega0_;
  Eigen::Vector3d omega_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__ANGULAR_INTEGRATION_HPP_
