// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#ifndef DRAKE_ROS_SYSTEMS__UTILITIES__NAME_CONVENTIONS_HPP_
#define DRAKE_ROS_SYSTEMS__UTILITIES__NAME_CONVENTIONS_HPP_

#include <drake/geometry/scene_graph_inspector.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <string>
#include <unordered_set>


namespace drake_ros_systems
{
namespace utilities
{

std::string
GetTfFrameName(
  const drake::geometry::SceneGraphInspector<double> & inspector,
  const std::unordered_set<const drake::multibody::MultibodyPlant<double> *> & plants,
  const drake::geometry::FrameId & frame_id);

std::string
GetTfFrameName(
  const drake::geometry::SceneGraphInspector<double> & inspector,
  const std::unordered_set<const drake::multibody::MultibodyPlant<double> *> & plants,
  const drake::geometry::GeometryId & geometry_id);

std::string
GetMarkerNamespace(
  const drake::geometry::SceneGraphInspector<double> & inspector,
  const std::unordered_set<const drake::multibody::MultibodyPlant<double> *> & plants,
  const drake::geometry::GeometryId & geometry_id);

}  // namespace utilities
}  // namespace drake_ros_systems

#endif  // DRAKE_ROS_SYSTEMS__UTILITIES__NAME_CONVENTIONS_HPP_
