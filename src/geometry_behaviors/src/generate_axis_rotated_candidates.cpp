// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "geometry_behaviors/generate_axis_rotated_candidates.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include "geometry_behaviors/axis_rotated_candidate_generation.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{
constexpr auto kPortIDInputPose = "input_pose";
constexpr auto kPortIDRotationAxis = "rotation_axis";
constexpr auto kPortIDAnglesDeg = "angles_deg";
constexpr auto kPortIDCandidates = "candidates";

const std::unordered_set<std::string> kValidAxes{ "+X", "-X", "X", "+Y", "-Y", "Y", "+Z", "-Z", "Z" };

// Map an axis name to its body-frame unit vector. Accepts "+X"/"-X"/"X" (= "+X")
// and the same for Y/Z. Returns nullopt for any other input.
std::optional<Eigen::Vector3d> axisFromString(const std::string& s)
{
  if (s == "+X" || s == "X")
    return Eigen::Vector3d::UnitX();
  if (s == "-X")
    return -Eigen::Vector3d::UnitX();
  if (s == "+Y" || s == "Y")
    return Eigen::Vector3d::UnitY();
  if (s == "-Y")
    return -Eigen::Vector3d::UnitY();
  if (s == "+Z" || s == "Z")
    return Eigen::Vector3d::UnitZ();
  if (s == "-Z")
    return -Eigen::Vector3d::UnitZ();
  return std::nullopt;
}

// Parse a semicolon-separated list of decimal angles in degrees, e.g. "45;-45;90;-90".
// Whitespace around tokens is tolerated; empty tokens are skipped. Returns nullopt
// on any token that fails to parse.
std::optional<std::vector<double>> parseAnglesDeg(const std::string& s)
{
  std::vector<double> out;
  std::istringstream iss(s);
  std::string token;
  while (std::getline(iss, token, ';'))
  {
    const auto first = token.find_first_not_of(" \t");
    const auto last = token.find_last_not_of(" \t");
    if (first == std::string::npos)
      continue;
    const std::string trimmed = token.substr(first, last - first + 1);
    try
    {
      out.push_back(std::stod(trimmed));
    }
    catch (...)
    {
      return std::nullopt;
    }
  }
  return out;
}
}  // namespace

namespace geometry_behaviors
{

GenerateAxisRotatedCandidates::GenerateAxisRotatedCandidates(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList GenerateAxisRotatedCandidates::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(
            kPortIDInputPose, "{input_pose}",
            "Base pose. Each output candidate is this pose rotated about its own body-frame "
            "rotation_axis by one of the supplied angles. Position and frame_id are unchanged."),
        BT::InputPort<std::string>(kPortIDRotationAxis, "{rotation_axis}",
                                   "Body-frame axis of input_pose to rotate around (+X/-X/+Y/-Y/+Z/-Z; X/Y/Z = "
                                   "+X/+Y/+Z)."),
        BT::InputPort<std::string>(kPortIDAnglesDeg, "{angles_deg}",
                                   "Semicolon-separated list of rotation angles in degrees, e.g. \"45;-45;90;-90\". "
                                   "Each angle produces one candidate, in list order."),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDCandidates, "{candidates}",
                                                                     "Vector of candidate poses (input_pose with each "
                                                                     "angle applied around rotation_axis).") });
}

BT::KeyValueVector GenerateAxisRotatedCandidates::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Generates a list of pose candidates from a single input pose, each rotated by a different "
             "angle around the input pose's body-frame rotation_axis. Useful for trying yaw / twist "
             "variations of a base grasp pose under a Fallback / ForEachUntilSuccess." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" } };
}

BT::NodeStatus GenerateAxisRotatedCandidates::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDInputPose),
                                               getInput<std::string>(kPortIDRotationAxis),
                                               getInput<std::string>(kPortIDAnglesDeg));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "GenerateAxisRotatedCandidates: missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [input_pose, rotation_axis, angles_str] = ports.value();

  if (!kValidAxes.count(rotation_axis))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Invalid rotation_axis '" + rotation_axis + "'. Valid: +X, -X, +Y, -Y, +Z, -Z (X/Y/Z = +X/+Y/+Z).");
    return BT::NodeStatus::FAILURE;
  }

  const auto axis_opt = axisFromString(rotation_axis);
  if (!axis_opt)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to map rotation_axis to vector.");
    return BT::NodeStatus::FAILURE;
  }
  const Eigen::Vector3d axis = axis_opt.value();

  const auto angles_opt = parseAnglesDeg(angles_str);
  if (!angles_opt)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to parse angles_deg '" + angles_str +
                                                                    "' (expected semicolon-separated decimals).");
    return BT::NodeStatus::FAILURE;
  }
  const auto& angles = angles_opt.value();
  if (angles.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "angles_deg parsed to empty list.");
    return BT::NodeStatus::FAILURE;
  }

  const auto candidates = rotateAroundBodyAxis(input_pose, axis, angles);
  setOutput(kPortIDCandidates, candidates);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
