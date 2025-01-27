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

#include <string>

// #include "lidar_feature_library/eigen.hpp"
#include "ros_dependent/ros_msg.hpp"

Matrix6d GetEigenCovariance(const std::array<double, 36> & covariance)
{
  Matrix6d matrix;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      matrix(i, j) = covariance[i * 6 + j];
    }
  }
  return matrix;
}

std::array<double, 36> FromEigenCovariance(const Matrix6d & covariance)
{
  std::array<double, 36> array;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      array[i * 6 + j] = covariance(i, j);
    }
  }
  return array;
}

Eigen::Isometry3d GetIsometry3d(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

geometry_msgs::msg::Pose MakePose(const Eigen::Isometry3d & pose)
{
  return tf2::toMsg(pose);
}

geometry_msgs::msg::PoseStamped MakePoseStamped(
  const Eigen::Isometry3d & pose, const rclcpp::Time & stamp, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = MakePose(pose);
  pose_stamped_msg.header.stamp = stamp;
  pose_stamped_msg.header.frame_id = frame_id;
  return pose_stamped_msg;
}

geometry_msgs::msg::PoseWithCovariance MakePoseWithCovariance(
  const Eigen::Isometry3d & pose,
  const Matrix6d & covariance)
{
  geometry_msgs::msg::PoseWithCovariance msg;
  msg.pose = MakePose(pose);
  msg.covariance = FromEigenCovariance(covariance);
  return msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped MakePoseWithCovarianceStamped(
  const Eigen::Isometry3d & pose,
  const Matrix6d & covariance,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.pose = MakePoseWithCovariance(pose, covariance);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  return msg;
}

geometry_msgs::msg::TransformStamped MakeTransformStamped(
  const Eigen::Isometry3d & transform,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  return msg;
}

geometry_msgs::msg::Point MakePoint(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point q;
  q.x = p(0);
  q.y = p(1);
  q.z = p(2);
  return q;
}

Eigen::Vector3d ToVector3d(const geometry_msgs::msg::Point & position)
{
  return Eigen::Vector3d(position.x, position.y, position.z);
}

Eigen::Vector3d ToVector3d(const geometry_msgs::msg::Vector3 & translation)
{
  return Eigen::Vector3d(translation.x, translation.y, translation.z);
}

Eigen::Quaterniond ToQuaterniond(const geometry_msgs::msg::Quaternion & rotation)
{
  return Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);
}

geometry_msgs::msg::Vector3 MakeVector3(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);
  return msg;
}

geometry_msgs::msg::Twist MakeTwist(
  const Eigen::Vector3d & linear,
  const Eigen::Vector3d & angular)
{
  geometry_msgs::msg::Twist msg;
  msg.linear = MakeVector3(linear);
  msg.angular = MakeVector3(angular);
  return msg;
}

geometry_msgs::msg::TwistStamped MakeTwistStamped(
  const Eigen::Vector3d & linear,
  const Eigen::Vector3d & angular,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.twist = MakeTwist(linear, angular);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  return msg;
}

visualization_msgs::msg::Marker InitLines(const rclcpp::Time & stamp, const std::string & frame_id)
{
  visualization_msgs::msg::Marker lines;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.header.stamp = stamp;
  lines.header.frame_id = frame_id;
  return lines;
}

void SetWidth(visualization_msgs::msg::Marker & lines, const float width)
{
  assert(0. <= width && width <= 1.);
  lines.scale.x = width;
}

void SetColor(
  visualization_msgs::msg::Marker & lines,
  const float r,
  const float g,
  const float b,
  const float a)
{
  assert(0. <= r && r <= 1.);
  assert(0. <= g && g <= 1.);
  assert(0. <= b && b <= 1.);
  assert(0. <= a && a <= 1.);

  lines.color.r = r;
  lines.color.g = g;
  lines.color.b = b;
  lines.color.a = a;
}

void AddLine(
  visualization_msgs::msg::Marker & lines,
  const Eigen::Vector3d & line_start,
  const Eigen::Vector3d & line_end)
{
  // The line list needs two points for each line
  lines.points.push_back(MakePoint(line_start));
  lines.points.push_back(MakePoint(line_end));
}
