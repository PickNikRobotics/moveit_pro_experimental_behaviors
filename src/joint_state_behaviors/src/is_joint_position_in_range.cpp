// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "joint_state_behaviors/is_joint_position_in_range.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <fmt/format.h>

#include <cmath>
#include <string>

namespace joint_state_behaviors
{
namespace internal
{
tl::expected<double, std::string> jointPosition(const sensor_msgs::msg::JointState& state, const std::string& joint_name)
{
  for (std::size_t i = 0; i < state.name.size(); ++i)
  {
    if (state.name[i] != joint_name)
    {
      continue;
    }
    if (i >= state.position.size())
    {
      return tl::make_unexpected(fmt::format("joint '{}' has no position in the state.", joint_name));
    }
    if (!std::isfinite(state.position[i]))
    {
      return tl::make_unexpected(fmt::format("joint '{}' position is not finite.", joint_name));
    }
    return state.position[i];
  }
  return tl::make_unexpected(fmt::format("joint '{}' is not in the state.", joint_name));
}
}  // namespace internal

IsJointPositionInRange::IsJointPositionInRange(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList IsJointPositionInRange::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene", "{planning_scene}",
                                                          "Scene whose robot state is read, typically straight from "
                                                          "GetCurrentPlanningScene."),
           BT::InputPort<std::string>("joint_name", "", "The joint to check."),
           BT::InputPort<double>("min_position", "Lowest position that counts as in range (inclusive)."),
           BT::InputPort<double>("max_position", "Highest position that counts as in range (inclusive)."),
           BT::OutputPort<double>("position", "{joint_position}", "The joint's current position.") };
}

BT::KeyValueVector IsJointPositionInRange::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Succeeds if one joint's current position (from a planning scene's robot state) is within "
             "[min_position, max_position], and fails otherwise. Outputs the position either way. Written as a grasp "
             "check for grippers whose close action succeeds whether or not it caught anything: the finger joint "
             "stalls part way on a part and runs to its end stop on nothing." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" } };
}

BT::NodeStatus IsJointPositionInRange::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<moveit_msgs::msg::PlanningScene>("planning_scene"),
                                               getInput<std::string>("joint_name"), getInput<double>("min_position"),
                                               getInput<double>("max_position"));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [scene, joint_name, min_position, max_position] = ports.value();
  if (min_position > max_position)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), fmt::format("min_position {} is above max_position {}.",
                                                                            min_position, max_position));
    return BT::NodeStatus::FAILURE;
  }

  const auto position = internal::jointPosition(scene.robot_state.joint_state, joint_name);
  if (!position.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), position.error());
    return BT::NodeStatus::FAILURE;
  }
  setOutput("position", *position);

  const bool in_range = *position >= min_position && *position <= max_position;
  const auto message = fmt::format("{} is {:.3f}, {} [{:.3f}, {:.3f}].", joint_name, *position,
                                   in_range ? "inside" : "OUTSIDE", min_position, max_position);
  if (in_range)
  {
    getBehaviorContext()->logger->publishInfoMessage(name(), message);
    return BT::NodeStatus::SUCCESS;
  }
  getBehaviorContext()->logger->publishWarnMessage(name(), message);
  return BT::NodeStatus::FAILURE;
}

}  // namespace joint_state_behaviors
