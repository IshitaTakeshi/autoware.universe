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

#ifndef ROS_DEPENDENT__MAP_RECEIVER_HPP_
#define ROS_DEPENDENT__MAP_RECEIVER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MapReceiver
{
public:
  MapReceiver(std::shared_ptr<rclcpp::Node> node, const std::string & topic_name);

  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr);

  bool IsAvailable() const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr Get() const;

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr_;
};

#endif  // ROS_DEPENDENT__MAP_RECEIVER_HPP_
