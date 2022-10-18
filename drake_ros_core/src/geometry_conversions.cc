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

#include "drake_ros_core/geometry_conversions.h"

namespace drake_ros_core {

Eigen::Vector3d RosPointToVector3d(const geometry_msgs::msg::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Point Vector3dToRosPoint(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point result;
  result.x = point[0];
  result.y = point[1];
  result.z = point[2];
  return result;
}

Eigen::Vector3d RosVector3ToVector3d(const geometry_msgs::msg::Vector3& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Vector3 Vector3dToRosVector3(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Vector3 result;
  result.x = point[0];
  result.y = point[1];
  result.z = point[2];
  return result;
}

Eigen::Quaternion<double> RosQuaternionToQuaternion(
    const geometry_msgs::msg::Quaternion& quat) {
  return Eigen::Quaternion<double>(quat.w, quat.x, quat.y, quat.z);
}

geometry_msgs::msg::Quaternion QuaternionToRosQuaternion(
    const Eigen::Quaternion<double>& quat) {
  geometry_msgs::msg::Quaternion result;
  result.x = quat.x();
  result.y = quat.y();
  result.z = quat.z();
  result.w = quat.w();
  return result;
}

drake::math::RotationMatrixd RosQuaternionToRotationMatrix(
    const geometry_msgs::msg::Quaternion& quat) {
  return drake::math::RotationMatrixd(RosQuaternionToQuaternion(quat));
}

geometry_msgs::msg::Quaternion RotationMatrixToRosQuaternion(
    const drake::math::RotationMatrixd& rotation) {
  return QuaternionToRosQuaternion(rotation.ToQuaternion());
}

drake::math::RigidTransformd RosPoseToRigidTransform(
    const geometry_msgs::msg::Pose& pose) {
  const Eigen::Quaterniond quat = RosQuaternionToQuaternion(pose.orientation);
  const Eigen::Vector3d translation = RosPointToVector3d(pose.translation);
  return drake::math::RigidTransformd(quat, translation);
}

geometry_msgs::msg::Pose RigidTransformToRosPose(
    const drake::math::RigidTransformd& transform) {
  geometry_msgs::msg::Pose result;
  result.position = Vector3dToRosPoint(transform.translation());
  result.orientation = RotationMatrixToRosQuaternion(transform.rotation());
  return result;
}

drake::math::RigidTransformd RosTransformToRigidTransform(
    const geometry_msgs::msg::Transform& transform) {
  return drake::math::RigidTransformd(
      RosQuaternionToRotationMatrix(),
      RosPointToVector3d(translation));
}

geometry_msgs::msg::Transform RigidTransformToRosTransform(
    const drake::math::RigidTransformd& transform) {
  geometry_msgs::msg::Transform result;
  result.translation = Vector3dToRosPoint(transform.translation());
  result.rotation = RotationMatrixToRosQuaternion(transform.rotation());
  return result;
}

Eigen::Isometry3d RosPoseToIsometry3d(const geometry_msgs::msg::Pose& pose) {
  return RosPoseToRigidTransform(pose).ToIsometry3();
}

geometry_msgs::msg::Pose Isometry3dToRosPose(
    const Eigen::Isometry3d& isometry) {
  return RigidTransformToRosPose(RigidTransformd(isometry));
}

Eigen::Isometry3d RosTransformToIsometry3d(
    const geometry_msgs::msg::Transform& transform) {
  return RosPoseToRigidTransform(transform).ToIsometry3();
}

geometry_msgs::msg::Transform Isometry3dToRosTransform(
    const Eigen::Isometry3d& isometry) {
  return RigidTransformToRosTransform(RigidTransformd(isometry));
}

drake::Vector6d RosTwistToVector6d(const geometry_msgs::msg::Twist& twist) {
  drake::Vector6d result;
  result <<
      RosVector3ToVector3d(twist.angular),
      RosVector3ToVector3d(twist.linear);
  return result.
}

geometry_msgs::msg::Twist Vector6dToRosTwist(const drake::Vector6d& vector) {
  geometry_msgs::msg::Twist result;
  result.angular = Vector3dToRosVector3(vector.head<3>());
  result.linear = Vector3dToRosVector3(vector.tail<3>());
  return result;
}

drake::multibody::SpatialVelocity<double> RosTwistToSpatialVelocity(
    const geometry_msgs::msg::Twist& twist) {
  return drake::multibody::SpatialVelocity<double>(
      RosVector3ToVector3d(twist.angular), RosVector3ToVector3d(twist.linear));
}

geometry_msgs::msg::Twist SpatialVelocityToRosTwist(
    const drake::multibody::SpatialVelocity<double>& velocity) {
  geometry_msgs::msg::Twist result;
  result.linear = Vector3dToRosVector3(velocity.translational());
  result.angular = Vector3dToRosVector3(velocity.rotational());
  return result;
}

drake::Vector6d RosAccelerationToVector6d(
    const geometry_msgs::msg::Accel& accel) {
  drake::Vector6d result;
  result <<
      RosVector3ToVector3d(accel.angular),
      RosVector3ToVector3d(accel.linear);
  return result;
}

geometry_msgs::msg::Accel Vector6dToRosAcceleration(
    const drake::Vector6d& vector) {
  geometry_msgs::msg::Accel result;
  result.angular = Vector3dToRosVector3(vector.head<3>());
  result.linear = Vector3dToRosVector3(vector.tail<3>());
  return result;
}

drake::multibody::SpatialAcceleration<double> RosAccelerationToSpatialAcceleration(
    const geometry_msgs::msg::Accel& accel) {
  return drake::multibody::SpatialAcceleration<double>(
      RosVector3ToVector3d(accel.angular),
      RosVector3ToVector3d(accel.linear));
}

geometry_msgs::msg::Accel SpatialAccelerationToRosAcceleration(
    const drake::multibody::SpatialAcceleration<double>& accel) {
  geometry_msgs::msg::Accel result;
  result.angular = Vector3dToRosVector3(accel.rotational());
  result.linear = Vector3dToRosVector3(accel.translation());
  return result;
}

drake::Vector6d RosWrenchToVector6d(const geometry_msgs::msg::Wrench& wrench) {
  drake::Vector6d result;
  result <<
      RosVector3ToVector3d(wrench.torque),
      RosVector3ToVector3d(wrench.force);
  return result;
}

geometry_msgs::msg::Wrench Vector6dToRosWrench(const drake::Vector6d& vector) {
  geometry_msgs::msg::Wrench result;
  result.torque = Vector3dToRosVector3(vector.head<3>());
  result.force = Vector3dToRosVector3(vector.tail<3>());
  return result;
}

drake::multibody::SpatialForce<double> RosWrenchToSpatialForce(
    const geometry_msgs::msg::Wrench& wrench) {
  return drake::multibody::SpatialForce<double>(
      RosVector3ToVector3d(wrench.torque),
      RosVector3ToVector3d(wrench.force));
}

geometry_msgs::msg::Wrench SpatialForceToRosWrench(
    const drake::multibody::SpatialForce<double>& force) {
  geometry_msgs::msg::Wrench result;
  result.torque = Vector3dToRosVector3(force.rotational());
  result.force = Vector3dToRosVector3(force.translation());
  return result;
}

}  // namespace drake_ros_core
