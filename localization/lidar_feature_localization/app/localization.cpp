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

#include "lidar_feature_localization/localizer.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/subscriber.hpp"

#include "lidar_feature_extraction/edge_surface_extraction.hpp"
#include "lidar_feature_extraction/hyper_parameter.hpp"
#include "lidar_feature_extraction/ring.hpp"

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "lidar_feature_library/warning.hpp"

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/surface.hpp"

using Odometry = nav_msgs::msg::Odometry;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;

double Nanoseconds(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds());
}

Matrix6d MakeCovariance()
{
  Matrix6d covariance;
  covariance <<
    1.0, 0, 0, 0, 0, 0,
    0, 1.0, 0, 0, 0, 0,
    0, 0, 1.0, 0, 0, 0,
    0, 0, 0, 1.0, 0, 0,
    0, 0, 0, 0, 1.0, 0,
    0, 0, 0, 0, 0, 1.0;
  return covariance;
}

double ComputeX(const Eigen::Vector3d & w, const double & y, const double & z)
{
  return -(1.0 / w(0)) * (w(1) * y + w(2) * z + 1);
}

double ComputeY(const Eigen::Vector3d & w, const double & z, const double & x)
{
  return -(1.0 / w(1)) * (w(2) * z + w(0) * x + 1);
}

double ComputeZ(const Eigen::Vector3d & w, const double & x, const double & y)
{
  return -(1.0 / w(2)) * (w(0) * x + w(1) * y + 1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneYZ(
  const Eigen::Vector3d & w, const Eigen::Vector3d & mu, const int n)
{
  const double scale = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (int iy = n; iy <= n; iy++) {
    for (int iz = n; iz <= n; iz++) {
      const double y = mu(1) + static_cast<double>(iy) * scale;
      const double z = mu(2) + static_cast<double>(iz) * scale;
      const double x = ComputeX(w, y, z);
      plane->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneZX(
  const Eigen::Vector3d & w, const Eigen::Vector3d & mu, const int n)
{
  const double scale = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (int iz = n; iz <= n; iz++) {
    for (int ix = n; ix <= n; ix++) {
      const double z = mu(2) + static_cast<double>(iz) * scale;
      const double x = mu(0) + static_cast<double>(ix) * scale;
      const double y = ComputeY(w, z, x);
      plane->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneXY(
  const Eigen::Vector3d & w, const Eigen::Vector3d & mu, const int n)
{
  const double scale = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (int ix = n; ix <= n; ix++) {
    for (int iy = n; iy <= n; iy++) {
      const double x = mu(0) + static_cast<double>(ix) * scale;
      const double y = mu(1) + static_cast<double>(iy) * scale;
      const double z = ComputeZ(w, x, y);
      plane->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GeneratePlane(const Eigen::MatrixXd & X)
{
  const int n = 10;
  const Eigen::Vector3d w = EstimatePlaneCoefficients(X);
  const Eigen::Vector3d mu = X.colwise().mean();

  if (!CheckPointsDistributeAlongPlane(X, w)) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>());
    return empty;
  }

  if (w(0) >= w(1) && w(0) >= w(2)) {
    return PlaneYZ(w, mu, n);
  }

  if (w(1) >= w(2) && w(1) >= w(0)) {
    return PlaneZX(w, mu, n);
  }

  if (w(2) >= w(0) && w(2) >= w(1)) {
    return PlaneXY(w, mu, n);
  }

  assert(false);
  return nullptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetSurfaceCloud(
  const std::shared_ptr<Surface> & surface,
  const Eigen::Isometry3d & point_to_map,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan)
{
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(point_to_map, surface_scan);

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  for (const pcl::PointXYZ & query : *transformed) {
    const pcl::PointCloud<pcl::PointXYZ> neighbors = surface->NearestKSearch(query);
    const Eigen::MatrixXd X = GetXYZ(neighbors);
    *plane += *GeneratePlane(X);
  }
  return plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetEdgeCloud(
  const std::shared_ptr<Edge> & edge,
  const Eigen::Isometry3d & point_to_map,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan)
{
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(point_to_map, edge_scan);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const pcl::PointXYZ & query : *transformed) {
    const auto [mean, covariance] = edge->NeighborMeanAndCovariance(query);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ed = EigenDecomposition(covariance);
    const Eigen::Vector3d eigenvalues = ed.eigenvalues();

    if (IsEdge(eigenvalues)) {
      continue;
    }

    const Eigen::Vector3d principal = ed.eigenvectors().col(2);

    for (int i = -20; i <= 20; i++) {
      const Eigen::Vector3d v = mean + 0.1 * static_cast<double>(i) * principal;
      const pcl::PointXYZ p = PointXYZToVector::ToPoint(v);
      cloud->push_back(p);
    }
  }
  return cloud;
}

template<typename PointType>
class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map)
  : Node("lidar_feature_extraction"),
    params_(this),
    edge_(std::make_shared<Edge>(edge_map, params_.n_edge_neighbors)),
    surface_(std::make_shared<Surface>(surface_map, params_.n_surface_neighbors)),
    localizer_(edge_, surface_, params_.max_iter, params_.huber_k),
    tf_broadcaster_(*this),
    warning_(this),
    extraction_(params_),
    cloud_subscriber_(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", QOS_BEST_EFFORT_VOLATILE,
        std::bind(&LocalizationNode::Callback, this, std::placeholders::_1))),
    optimization_start_pose_subscriber_(
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "optimization_start_pose", QOS_BEST_EFFORT_VOLATILE,
        std::bind(&LocalizationNode::OptimizationStartPoseCallback, this, std::placeholders::_1))),
    edge_publisher_(this->create_publisher<PointCloud2>("edge_features", 10)),
    surface_publisher_(this->create_publisher<PointCloud2>("surface_features", 10)),
    target_edge_publisher_(this->create_publisher<PointCloud2>("target_edge_features", 10)),
    target_surface_publisher_(this->create_publisher<PointCloud2>("target_surface_features", 10)),
    pose_publisher_(this->create_publisher<PoseStamped>("estimated_pose", 10)),
    pose_with_covariance_publisher_(
      this->create_publisher<PoseWithCovarianceStamped>("estimated_pose_with_covariance", 10))
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~LocalizationNode() {}

private:
  void OptimizationStartPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr stamped_pose)
  {
    this->SetOptimizationStartPose(stamped_pose->header.stamp, stamped_pose->pose.pose);
  }

  void SetOptimizationStartPose(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
  {
    const double msg_stamp_nanosec = Nanoseconds(stamp);
    warning_.Info(fmt::format("Received a prior pose of time {}", msg_stamp_nanosec));
    prior_poses_.Insert(msg_stamp_nanosec, GetIsometry3d(pose));
  }

  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    warning_.Info("Received a cloud message");
    if (prior_poses_.Size() == 0) {
      warning_.Warn("Received a cloud message but there's no pose in the prior queue");
      return;
    }

    const auto input_cloud = GetPointCloud<PointType>(*cloud_msg);

    if (!input_cloud->is_dense) {
      warning_.Error("Point cloud must be in the dense format");
      rclcpp::shutdown();
    }

    if (!RingIsAvailable(cloud_msg->fields)) {
      warning_.Error("Ring channel could not be found");
      rclcpp::shutdown();
    }

    const auto [edge, surface] = extraction_.Run(input_cloud);

    const std::string lidar_frame = "lidar_feature_base_link";
    const rclcpp::Time stamp = cloud_msg->header.stamp;
    edge_publisher_->publish(ToRosMsg<pcl::PointXYZ>(edge, stamp, lidar_frame));
    surface_publisher_->publish(ToRosMsg<pcl::PointXYZ>(surface, stamp, lidar_frame));

    const double msg_stamp_nanosec = Nanoseconds(stamp);
    const auto [prior_stamp_nanosec, prior] = prior_poses_.GetClosest(msg_stamp_nanosec);
    prior_poses_.RemoveOlderThan(msg_stamp_nanosec + 1e9);  // 1e9 msg_stamp_nanosec = 1s

    localizer_.Init(prior);
    localizer_.Update(std::make_tuple(edge, surface));
    const Eigen::Isometry3d pose = localizer_.Get();

    const auto target_edges = TargetEdgeCloud(edge_, pose, edge);
    const auto target_surfaces = TargetSurfaceCloud(surface_, pose, surface);
    target_edge_publisher_->publish(ToRosMsg<pcl::PointXYZ>(target_edges, stamp, "map"));
    target_surface_publisher_->publish(ToRosMsg<pcl::PointXYZ>(target_surfaces, stamp, "map"));

    pose_publisher_->publish(MakePoseStamped(pose, stamp, "map"));

    const Matrix6d covariance = MakeCovariance();

    pose_with_covariance_publisher_->publish(
      MakePoseWithCovarianceStamped(pose, covariance, stamp, "map")
    );

    tf_broadcaster_.sendTransform(
      MakeTransformStamped(pose, stamp, "map", "lidar_feature_base_link")
    );

    warning_.Info("Pose update done");
  }

  const HyperParameters params_;
  const std::shared_ptr<Edge> edge_;
  const std::shared_ptr<Surface> surface_;
  Localizer localizer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  StampSortedObjects<Eigen::Isometry3d> prior_poses_;
  const Warning warning_;
  const EdgeSurfaceExtraction<PointType> extraction_;
  const rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr optimization_start_pose_subscriber_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr edge_publisher_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr surface_publisher_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr target_edge_publisher_;
  const rclcpp::Publisher<PointCloud2>::SharedPtr target_surface_publisher_;
  const rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher_;
  const rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;
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
  const std::string edge_map_path = "/map/edge.pcd";
  const std::string surface_map_path = "/map/surface.pcd";

  if (!CheckMapPathExists(edge_map_path)) {
    return -1;
  }

  if (!CheckMapPathExists(surface_map_path)) {
    return -1;
  }

  const auto edge_map = LoadPointCloud(edge_map_path);
  const auto surface_map = LoadPointCloud(surface_map_path);

  using Node = LocalizationNode<PointXYZIRADT>;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node>(edge_map, surface_map));
  rclcpp::shutdown();
  return 0;
}
