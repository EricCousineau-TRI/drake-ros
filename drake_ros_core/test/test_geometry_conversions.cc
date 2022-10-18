// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "drake_ros_core/geometry_conversions.h"

namespace drake_ros_core {
namespace {

// N.B. For the purpose of testing type conversions, we should use explicit
// typing (avoiding auto).

// Vector / Translation.

Eigen::Vector3d MakeDummyVector3() {
  // Arbitrary value with distinct elements.
  return Eigen::Vector3d(1.0, 2.0, 3.0);
}

geometry_msgs::msg::Point MakeDummyRosPoint() {
  // Represents same quantity as MakeDummyVector3.
  geometry_msgs::msg::Point message;
  message.x = 1.0;
  message.y = 2.0;
  message.z = 3.0;
  return message;
}

TEST(GeometryConversions, Point) {
  const geometry_msgs::msg::Point message = MakeDummyRosPoint();
  const Eigen::Vector3d value_expected = MakeDummyVector3();
  const Eigen::Vector3d value = RosPointToVector3d(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, Vector3dToRosPoint(value));
}

geometry_msgs::msg::Vector3 MakeDummyRosVector3() {
  // Represents same quantity as MakeDummyVector3.
  geometry_msgs::msg::Vector3 message;
  message.x = 1.0;
  message.y = 2.0;
  message.z = 3.0;
  return message;
}

TEST(GeometryConversions, Vector3) {
  const geometry_msgs::msg::Vector3 message = MakeDummyRosVector3();
  const Eigen::Vector3d value_expected = MakeDummyVector3();
  const Eigen::Vector3d value = RosVector3ToVector3d(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, Vector3dToRosVector3(value));
}

// Orientation.

Eigen::Quatnerniond MakeDummyQuaternion() {
  // This corresponds to RollPitchYaw(np.deg2rad([90, 0, 0]))
  const double w = 0.5, x = 0.5, y = 0.0, z = 0.0;
  return Eigen::Quatneriond(w, x, y, z);
}

geometry_msgs::msg::Quaternion MakeDummyRosQuaternion() {
  // Represents same quantity as MakeDummyQuaternion.
  geometry_msgs::msg::Quaternion message;
  message.w = 0.5;
  message.x = 0.5;
  message.y = 0.0;
  message.z = 0.0;
  return message;
}

TEST(GeometryConversions, Quaternion) {
  const geometry_msgs::msg::Quaternion message = MakeDummyRosQuaternion();
  const Eigen::Quaterniond value_expected = MakeDummyQuaternion();
  const Eigen::Quaterniond value = RosQuaternionToQuaternion(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, QuaternionToRosQuaternion(value));
}

drake::math::RotationMatrixd MakeDummyRotationMatrixd() {
  // This corresponds to RollPitchYaw(np.deg2rad([90, 0, 0]))
  Eigen::Matrix3d R;
  R <<
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0,
      0.0, 1.0, 0.0;
  return drake::math::RotationMatrixd(R);
}

TEST(GeometryConversions, RotationMatrix) {
  const geometry_msgs::msg::Quaternion message = MakeDummyRosQuaternion();
  const drake::math::RotationMatrixd value_expected =
      MakeDummyRotationMatrixd();
  const drake::math::RotationMatrixd value =
      RosQuaternionToRotationMatrixd(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, RotationMatrixdToRosQuaternion(value));
}

// Pose.

drake::math::RigidTransformd MakeDummyRigidTransform() {
  return drake::math::RigidTransformd(
      MakeDummyQuaternion(),
      MakeDummyVector3());
}

geometry_msgs::msg::Pose MakeDummyRosPose() {
  // Represents same quantity as MakeDummyRigidTransform.
  geometry_msgs::msg::Pose message;
  message.position = MakeDummyRosPoint();
  message.orientation = MakeDummyRosQuaternion();
  return message;
}

TEST(GeometryConversions, Pose) {
  const geometry_msgs::msg::Pose message = MakeDummyRosPose();
  const drake::math::RigidTransformd value_expected =
      MakeDummyRigidTransform();
  const drake::math::RigidTransformd value =
      RosPoseToRigidTransform(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, RigidTransformToRosPose(value));

  // Test Isometry3 flavoring.
  EXPECT_EQ(message, Isometry3ToRosPose(value.ToIsometry3()));
  EXPECT_EQ(value.ToIsometry3(), RosPoseToIsometry3(message));
}

geometry_msgs::msg::Transform MakeDummyRosTransform() {
  // Represents same quantity as MakeDummyRigidTransform.
  geometry_msgs::msg::Transform message;
  message.position = MakeDummyRosPoint();
  message.orientation = MakeDummyRosQuaternion();
  return message;
}

TEST(GeometryConversions, Transform) {
  const geometry_msgs::msg::Transform message = MakeDummyRosTransform();
  const drake::math::RigidTransformd value_expected =
      MakeDummyRigidTransform();
  const drake::math::RigidTransformd value =
      RosTransformToRigidTransform(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, RigidTransformToRosTransform(value));

  // Test Isometry3 flavoring.
  EXPECT_EQ(message, Isometry3ToRosTransform(value.ToIsometry3()));
  EXPECT_EQ(value.ToIsometry3(), RosTransformToIsometry3(message));
}

// General Spatial Vectors.

// We should to distinguish between translational and rotational values.

Eigen::Vector3d MakeDummyVector3ForRotation() {
  return 0.1 * MakeDummyVector3();
}

Eigen::Vector3d MakeDummyVector3ForTranslation() {
  return MakeDummyVector3();
}

geometry_msgs::msg::Vector3 MakeDummyRosVector3ForRotation() {
  // Represents same quantity as MakeDummyVector3ForRotation.
  geometry_msgs::msg::Vector3 value;
  value.x = 0.1;
  value.y = 0.2;
  value.z = 0.3;
  return value;
}

geometry_msgs::msg::Vector3 MakeDummyRosVector3ForTranslation() {
  // Represents same quantity as MakeDummyVector3ForTranslation.
  geometry_msgs::msg::Vector3 value;
  value.x = 1.0;
  value.y = 2.0;
  value.z = 3.0;
  return value;
}

drake::Vector6d MakeDummyVector6() {
  drake::Vector6d value;
  value <<
      MakeDummyVector3ForRotation(),
      MakeDummyVector3ForTranslation();
  return value;
}

// Spatial Velocity.

drake::math::SpatialVelocity<double> MakeDummySpatialVelocity() {
  // Corresponds to same value as MakeDummyVector6.
  return drake::math::SpatialVelocity<double>(
      MakeDummyVector3ForRotation(),
      MakeDummyVector3ForTranslation());
}

geometry_msgs::msg::Twist MakeDummyRosTwist() {
  // Represents same quantity as MakeDummySpatialVelocity.
  geometry_msgs::msg::Twist value;
  value.angular = MakeDummyRosVector3ForRotation();
  value.linear = MakeDummyRosVector3ForTranslation();
  return value;
}

TEST(GeometryConversions, Twist) {
  const geometry_msgs::msg::Twist message = MakeDummyRosTwist();
  const drake::multibody::SpatialVelocity<double> value_expected =
      MakeDummySpatialVelocity();
  const drake::multibody::SpatialVelocity<double> value =
      RosTwistToSpatialVelocity(message);
  EXPECT_EQ(value, value_expected);
  EXPECT_EQ(message, SpatialVelocityToRosTwist(value));
}

// TODO: Acceleration, Force.

}  // namespace
}  // namespace drake_ros_core
