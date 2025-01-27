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

#ifndef LIDAR_FEATURE_LOCALIZATION__STAMP_SORTED_OBJECTS_HPP_
#define LIDAR_FEATURE_LOCALIZATION__STAMP_SORTED_OBJECTS_HPP_

#include <fmt/core.h>

#include <map>
#include <mutex>
#include <tuple>

template<typename Object>
class StampSortedObjects
{
public:
  StampSortedObjects()
  {
  }

  void Insert(const double timestamp, const Object & object)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    objects_[timestamp] = object;
  }

  std::tuple<double, Object> GetClosest(const double timestamp) const
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // g1 is the first element in map that satisfies g1 >= timestamp
    const auto g1 = objects_.lower_bound(timestamp);

    if (g1 == objects_.end()) {
      const auto last = std::prev(objects_.end());
      return std::make_tuple(last->first, last->second);
    }

    if (g1 == objects_.begin()) {
      const auto first = objects_.begin();
      return std::make_tuple(first->first, first->second);
    }

    const auto g0 = std::prev(g1);
    const auto [time0, object0] = *g0;
    const auto [time1, object1] = *g1;

    const double d0 = timestamp - time0;
    const double d1 = time1 - timestamp;
    return d0 < d1 ?
           std::make_tuple(time0, object0) :
           std::make_tuple(time1, object1);
  }

  std::tuple<double, Object> GetFirst() const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return *objects_.begin();
  }

  std::tuple<double, Object> GetLast() const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return *std::prev(objects_.end());
  }

  double GetFirstTimestamp() const
  {
    return std::get<0>(this->GetFirst());
  }

  double GetLastTimestamp() const
  {
    return std::get<0>(this->GetLast());
  }

  std::tuple<double, Object> GetPrev(const double timestamp) const
  {
    std::lock_guard<std::mutex> guard(mutex_);

    const double first = std::get<0>(*objects_.begin());
    if (timestamp <= first) {
      throw std::invalid_argument(fmt::format("There's no element before {}", first));
    }
    // the first element in map that satisfies g >= timestamp
    const auto g = objects_.lower_bound(timestamp);
    return *std::prev(g);
  }

  std::tuple<double, Object> GetNext(const double timestamp) const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto last = *std::prev(objects_.end());
    const double last_time = std::get<0>(last);
    if (last_time <= timestamp) {
      throw std::invalid_argument(fmt::format("There's no element after {}", last_time));
    }
    return *objects_.upper_bound(timestamp);
  }

  size_t Size() const
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return objects_.size();
  }

  void RemoveLargerThan(const double timestamp)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // g is the first element in map that satisfies timestamp < g
    const auto g = objects_.upper_bound(timestamp);

    if (g == objects_.end()) {
      return;
    }

    objects_.erase(g, objects_.end());
  }

  void RemoveSmallerThan(const double timestamp)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // g is the first element in map that satisfies timestamp <= g
    const auto g = objects_.lower_bound(timestamp);

    if (g == objects_.begin()) {
      return;
    }

    objects_.erase(objects_.begin(), g);
  }

private:
  mutable std::mutex mutex_;
  std::map<double, Object> objects_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__STAMP_SORTED_OBJECTS_HPP_
