// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "grasping_behaviors/generate_grasp_candidates.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "grasping_behaviors/grasp_candidate_generation.hpp"

namespace
{
constexpr auto kPortIDGraspableObject = "graspable_object";
constexpr auto kPortIDPointCloud = "point_cloud";
constexpr auto kPortIDNumCylinderCandidates = "num_cylinder_candidates";
constexpr auto kPortIDNumSphereRingCandidates = "num_sphere_ring_candidates";
constexpr auto kPortIDSphereTiltAngleDegrees = "sphere_tilt_angle_degrees";
constexpr auto kPortIDGraspCandidates = "grasp_candidates";

inline constexpr auto kDescription = R"(
    <p>Generates grasp candidate poses for detected objects based on their fitted primitive shape.</p>
    <ul>
      <li><b>Cylinder:</b> Two poses centered on the flat end whose outward normal points most upward in world Z
          (tried first) — one with Z along the outward end normal, one flipped 180° to point along the inward
          end normal toward the cylinder's geometric center — followed by <code>num_cylinder_candidates</code>
          poses evenly spaced around the circumference at mid-height, all with Z-axis pointing radially
          outward from the surface.</li>
      <li><b>Box:</b> Up to 3 poses, one centered on each of the most accessible faces
          (selected by choosing the more upward-facing face from each opposite pair).</li>
      <li><b>Sphere:</b> One pose at the topmost point plus <code>num_sphere_ring_candidates</code>
          poses in a ring tilted <code>sphere_tilt_angle_degrees</code> from the vertical,
          all with Z-axis pointing radially outward.</li>
    </ul>
)";

}  // namespace

namespace grasping_behaviors
{

GenerateGraspCandidates::GenerateGraspCandidates(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList GenerateGraspCandidates::providedPorts()
{
  return {
    BT::InputPort<moveit_studio_vision_msgs::msg::GraspableObject>(kPortIDGraspableObject, "{graspable_object}",
                                                                   "A single graspable object from shape fitting."),
    BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}", "Input point cloud."),
    BT::InputPort<int>(kPortIDNumCylinderCandidates, 8, "Number of grasp candidates to generate around each cylinder."),
    BT::InputPort<int>(kPortIDNumSphereRingCandidates, 4,
                       "Number of grasp candidates in the tilted ring around each sphere."),
    BT::InputPort<double>(kPortIDSphereTiltAngleDegrees, 22.5,
                          "Tilt angle in degrees from the vertical for the sphere ring candidates."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDGraspCandidates, "{grasp_candidates}",
                                                                 "Grasp candidate poses on object surfaces."),
  };
}

BT::KeyValueVector GenerateGraspCandidates::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Grasp Planning" } };
}

BT::NodeStatus GenerateGraspCandidates::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<moveit_studio_vision_msgs::msg::GraspableObject>(kPortIDGraspableObject),
      getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required inputs: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [graspable_object, point_cloud_msg] = ports.value();
  const int num_cylinder_candidates = getInput<int>(kPortIDNumCylinderCandidates).value_or(8);
  const int num_sphere_ring_candidates = getInput<int>(kPortIDNumSphereRingCandidates).value_or(4);
  const double sphere_tilt_deg = getInput<double>(kPortIDSphereTiltAngleDegrees).value_or(22.5);

  const auto grasp_candidates = generateGraspCandidatesForPrimitive(graspable_object, num_cylinder_candidates,
                                                                    num_sphere_ring_candidates, sphere_tilt_deg);
  if (!grasp_candidates.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), grasp_candidates.error());
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDGraspCandidates, grasp_candidates.value());

  getBehaviorContext()->logger->publishInfoMessage(
      name(), "Generated " + std::to_string(grasp_candidates.value().size()) + " grasp candidates for " +
                  graspable_object.grasp_info.object_type + " object.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace grasping_behaviors
