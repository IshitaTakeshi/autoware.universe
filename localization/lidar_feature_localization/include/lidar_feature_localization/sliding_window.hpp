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

#ifndef LIDAR_FEATURE_LOCALIZATION__SLIDING_WINDOW_HPP_
#define LIDAR_FEATURE_LOCALIZATION__SLIDING_WINDOW_HPP_

#include <fmt/core.h>

#include <deque>
#include <stdexcept>
#include <tuple>


template<typename ObjectType>
class SlidingWindow
{
public:
  explicit SlidingWindow(const size_t window_size)
  : window_size_(window_size)
  {
  }

  void PushNew(const double timestamp, const ObjectType & object)
  {
    timestamps_.push_back(timestamp);
    objects_.push_back(object);
  }

  void PopOld()
  {
    timestamps_.pop_front();
    objects_.pop_front();
  }

  void Slide(const double timestamp, const ObjectType & object)
  {
    this->PushNew(timestamp, object);

    if (this->Size() <= window_size_) {
      return;
    }

    this->PopOld();
  }

  std::tuple<double, ObjectType> Get(const int index) const
  {
    this->ThrowInvalidArgumentIfOutOfRange(index);
    return std::make_tuple(timestamps_.at(index), objects_.at(index));
  }

  size_t Size() const
  {
    return timestamps_.size();
  }

  bool IsFilled() const
  {
    return this->Size() == window_size_;
  }

private:
  void ThrowInvalidArgumentIfOutOfRange(const int index) const
  {
    const int size = static_cast<int>(this->Size());

    if (index < 0 || size <= index) {
      const auto s = fmt::format("Index {} is out of range [0, {})", index, size);
      throw std::invalid_argument(s);
    }
  }

  const size_t window_size_;
  std::deque<double> timestamps_;
  std::deque<ObjectType> objects_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__SLIDING_WINDOW_HPP_
