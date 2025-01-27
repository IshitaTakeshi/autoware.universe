// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIDAR_FEATURE_LIBRARY__WARNING_HPP_
#define LIDAR_FEATURE_LIBRARY__WARNING_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>


class Warning
{
public:
  explicit Warning(rclcpp::Node * node)
  : node_(node)
  {
  }

  void Info(const std::string & message) const
  {
    RCLCPP_INFO(node_->get_logger(), message.c_str());
  }

  void Warn(const std::string & message) const
  {
    RCLCPP_WARN(node_->get_logger(), message.c_str());
  }

  void Error(const std::string & message) const
  {
    RCLCPP_ERROR(node_->get_logger(), message.c_str());
  }

  void WarnThrottle(
    const int duration_milliseconds,
    const std::string & message) const
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()),
      std::chrono::milliseconds(duration_milliseconds).count(),
      message.c_str());
  }

private:
  rclcpp::Node * node_;
};

#endif  // LIDAR_FEATURE_LIBRARY__WARNING_HPP_
