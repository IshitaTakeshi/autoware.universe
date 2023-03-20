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

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include "ros_dependent/map_receiver.hpp"


class MapTestSuite : public ::testing::Test
{
protected:
  void SetUp() {rclcpp::init(0, nullptr);}
  void TearDown() {(void)rclcpp::shutdown();}
};

TEST_F(MapTestSuite, MapReciever)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
  pcl_pointcloud.push_back(pcl::PointXYZ(1., 1., 1.));

  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(pcl_pointcloud, ros_pointcloud);

  rclcpp::NodeOptions node_options;
  const std::string topic_name = "pointcloud_map";
  auto node = std::make_shared<rclcpp::Node>("test_map_node", node_options);
  auto pub_pose = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    topic_name, rclcpp::QoS{1}.transient_local());

  MapReceiver receiver(node.get(), topic_name);
  EXPECT_FALSE(receiver.MapIsAvailable());

  for (int i = 0; i < 20; ++i) {
    pub_pose->publish(ros_pointcloud);
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(receiver.MapIsAvailable());
  const auto map_ptr = receiver.MapPtr();
  EXPECT_GT(map_ptr->size(), 0U);
  EXPECT_EQ(map_ptr->size(), pcl_pointcloud.size());
}
