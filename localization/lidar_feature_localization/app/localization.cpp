// Copyright 2022 Tixiao Shan, Takeshi Ishita
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

#include <fmt/core.h>

#include <rclcpp/rclcpp.hpp>

#include <rcpputils/filesystem_helper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <limits>
#include <memory>
#include <string>
#include <tuple>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "lidar_feature_extraction/edge_surface_extraction.hpp"
#include "lidar_feature_extraction/hyper_parameter.hpp"
#include "lidar_feature_extraction/ring.hpp"

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/qos.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "lidar_feature_library/warning.hpp"

#include "lidar_feature_localization/debug.hpp"
#include "lidar_feature_localization/localizer.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/stamp_sorted_objects.hpp"

using Odometry = nav_msgs::msg::Odometry;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using TwistStamped = geometry_msgs::msg::TwistStamped;


const char map_frame[] = "map";
const char lidar_frame[] = "lidar_feature_base_link";

inline double Nanoseconds(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds());
}

inline double Seconds(const rclcpp::Time & t)
{
  return 1e-9 * Nanoseconds(t);
}

Matrix6d MakeCovariance()
{
  Matrix6d covariance;
  covariance <<
    0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0.1;
  return covariance;
}

class DebugCloudPublisher
{
public:
  DebugCloudPublisher(
    rclcpp::Node * node,
    const std::shared_ptr<Edge> & edge,
    const std::shared_ptr<Surface> & surface)
  : edge_(edge), surface_(surface),
    edge_publisher_(node->create_publisher<PointCloud2>("edge_features", 10)),
    surface_publisher_(node->create_publisher<PointCloud2>("surface_features", 10)),
    target_edge_publisher_(node->create_publisher<PointCloud2>("target_edge_features", 10)),
    target_surface_publisher_(node->create_publisher<PointCloud2>("target_surface_features", 10))
  {
  }

  void Publish(
    const rclcpp::Time stamp,
    const Eigen::Isometry3d pose,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface) const
  {
    edge_publisher_->publish(ToRosMsg<pcl::PointXYZ>(edge, stamp, lidar_frame));
    surface_publisher_->publish(ToRosMsg<pcl::PointXYZ>(surface, stamp, lidar_frame));

    const auto target_edges = TargetEdgeCloud(edge_, pose, edge);
    const auto target_surfaces = TargetSurfaceCloud(surface_, pose, surface);
    target_edge_publisher_->publish(ToRosMsg<pcl::PointXYZ>(target_edges, stamp, map_frame));
    target_surface_publisher_->publish(ToRosMsg<pcl::PointXYZ>(target_surfaces, stamp, map_frame));
  }

private:
  const std::shared_ptr<Edge> edge_;
  const std::shared_ptr<Surface> surface_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr edge_publisher_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr surface_publisher_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr target_edge_publisher_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr target_surface_publisher_;
};

template<typename PointType>
class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & map)
  : Node("lidar_feature_extraction"),
    is_activated_(false),
    params_(this),
    edge_(std::make_shared<Edge>(map, params_.n_edge_neighbors)),
    surface_(std::make_shared<Surface>(map, params_.n_surface_neighbors)),
    debug_cloud_publisher_(this, edge_, surface_),
    localizer_(edge_, surface_, params_.max_iter, params_.huber_k),
    tf_broadcaster_(*this),
    warning_(this),
    extraction_(params_),
    cloud_subscriber_(
      this->create_subscription<PointCloud2>(
        "points_raw", QOS_BEST_EFFORT_VOLATILE,
        std::bind(&LocalizationNode::PointCloudCallback, this, std::placeholders::_1))),
    optimization_start_pose_subscriber_(
      this->create_subscription<PoseStamped>(
        "optimization_start_pose", QOS_BEST_EFFORT_VOLATILE,
        std::bind(&LocalizationNode::OptimizationStartPoseCallback, this, std::placeholders::_1))),
    pose_publisher_(this->create_publisher<PoseStamped>("estimated_pose", 10)),
    pose_with_covariance_publisher_(
      this->create_publisher<PoseWithCovarianceStamped>("estimated_pose_with_covariance", 10)),
    service_trigger_(
      this->create_service<std_srvs::srv::SetBool>(
        "trigger_node_srv",
        std::bind(
          &LocalizationNode::ServiceTrigger, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS().get_rmw_qos_profile()))
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~LocalizationNode() {}

private:
  void OptimizationStartPoseCallback(const PoseStamped::ConstSharedPtr stamped_pose)
  {
    this->SetOptimizationStartPose(stamped_pose->header.stamp, stamped_pose->pose);
  }

  void SetOptimizationStartPose(const rclcpp::Time & stamp, const Pose & pose)
  {
    const double msg_stamp_nanosec = Nanoseconds(stamp);
    warning_.Info(fmt::format("Received a prior pose of time {}", msg_stamp_nanosec));
    prior_poses_.Insert(msg_stamp_nanosec, GetIsometry3d(pose));
  }

  void PointCloudCallback(const PointCloud2::ConstSharedPtr cloud_msg)
  {
    // activate the node only after receiving the activation signal
    if (!is_activated_) {
      return;
    }

    warning_.Info("Received a cloud message");
    if (prior_poses_.Size() == 0) {
      warning_.Warn("Received a cloud message but there's no pose in the prior queue");
      return;
    }

    if (!RingIsAvailable(cloud_msg->fields)) {
      warning_.Error("Ring channel could not be found");
      rclcpp::shutdown();
    }

    const auto input_cloud = GetPointCloud<PointType>(*cloud_msg);

    if (!input_cloud->is_dense) {
      warning_.Error("Point cloud must be in the dense format");
      rclcpp::shutdown();
    }

    const auto [edge, surface] = extraction_.Run(input_cloud);

    const rclcpp::Time stamp = cloud_msg->header.stamp;

    const double msg_stamp_nanosec = Nanoseconds(stamp);
    const auto [prior_stamp_nanosec, prior] = prior_poses_.GetClosest(msg_stamp_nanosec);
    prior_poses_.RemoveOlderThan(msg_stamp_nanosec + 1e9);  // 1e9 msg_stamp_nanosec = 1s

    localizer_.Init(prior);
    localizer_.Update(std::make_tuple(edge, surface));
    const Eigen::Isometry3d pose = localizer_.Get();

    debug_cloud_publisher_.Publish(stamp, pose, edge, surface);

    pose_publisher_->publish(MakePoseStamped(pose, stamp, map_frame));

    const Matrix6d covariance = MakeCovariance();

    pose_with_covariance_publisher_->publish(
      MakePoseWithCovarianceStamped(pose, covariance, stamp, map_frame));

    tf_broadcaster_.sendTransform(MakeTransformStamped(pose, stamp, map_frame, lidar_frame));

    warning_.Info("Pose update done");
  }

  void ServiceTrigger(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    is_activated_ = req->data;
    res->success = true;
    warning_.Info("LOAM is activated");
  }

  bool is_activated_;
  const HyperParameters params_;
  const std::shared_ptr<Edge> edge_;
  const std::shared_ptr<Surface> surface_;
  const DebugCloudPublisher debug_cloud_publisher_;
  Localizer localizer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  StampSortedObjects<Eigen::Isometry3d> prior_poses_;
  const Warning warning_;
  const EdgeSurfaceExtraction<PointType> extraction_;
  const rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Subscription<PoseStamped>::SharedPtr optimization_start_pose_subscriber_;
  const rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher_;
  const rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;
  const rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_;
};

bool CheckMapPathExists(const std::string & map_path)
{
  bool exists = rcpputils::fs::exists(map_path);
  if (!exists) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lidar_feature_localization"),
      "Map %s does not exist!", map_path.c_str());
  }
  return exists;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPointCloud(const std::string & path)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  const int loaded = pcl::io::loadPCDFile(path, *cloud);
  if (loaded != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lidar_feature_localization"),
      "Failed to load %s", path.c_str());
  }
  return cloud;
}

int main(int argc, char * argv[])
{
  const std::string map_path = "/map/pointcloud_map.pcd";

  if (!CheckMapPathExists(map_path)) {
    return -1;
  }

  const auto map = LoadPointCloud(map_path);

  using Node = LocalizationNode<PointXYZIRADT>;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node>(map));
  rclcpp::shutdown();
  return 0;
}