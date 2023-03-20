// Copyright 2023 The Autoware Contributors
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

#include "ros_dependent/map_receiver.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <string>


MapReceiver::MapReceiver(rclcpp::Node * node, const std::string & topic_name)
: map_points_sub_(
    node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, rclcpp::QoS{1}.transient_local(),
      std::bind(&MapReceiver::Callback, this, std::placeholders::_1))),
  map_points_ptr_(new pcl::PointCloud<pcl::PointXYZ>())
{
}

void MapReceiver::Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  pcl::fromROSMsg(*map_points_msg_ptr, *(this->map_points_ptr_));
}

bool MapReceiver::MapIsAvailable() const
{
  return this->map_points_ptr_->size() > 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapReceiver::MapPtr() const
{
  return this->map_points_ptr_;
}
