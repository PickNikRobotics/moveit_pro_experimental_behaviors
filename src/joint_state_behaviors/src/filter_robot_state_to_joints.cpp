// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "joint_state_behaviors/filter_robot_state_to_joints.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <fmt/format.h>
#include <moveit_msgs/msg/robot_state.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace joint_state_behaviors
{

FilterRobotStateToJoints::FilterRobotStateToJoints(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList FilterRobotStateToJoints::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::RobotState>("input_state", "{input_state}",
                                                       "RobotState to filter, typically straight from "
                                                       "RetrieveWaypoint."),
           BT::InputPort<std::vector<std::string>>("joint_names", "",
                                                   "Semicolon-separated joints to keep, in the order they should "
                                                   "appear in the output."),
           BT::OutputPort<moveit_msgs::msg::RobotState>("output_state", "{output_state}",
                                                        "The same state carrying only the named joints.") };
}

BT::KeyValueVector FilterRobotStateToJoints::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Keeps only the named joints of a RobotState, so a waypoint saved with every joint on the robot can be "
             "handed to a planner that requires a subset of one planning group." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" } };
}

BT::NodeStatus FilterRobotStateToJoints::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<moveit_msgs::msg::RobotState>("input_state"),
                                                              getInput<std::vector<std::string>>("joint_names"));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [input_state, joint_names] = ports.value();

  if (joint_names.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "joint_names is empty; nothing to keep.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& src = input_state.joint_state;
  moveit_msgs::msg::RobotState out;
  out.joint_state.header = src.header;
  out.is_diff = input_state.is_diff;

  std::vector<std::string> missing;
  for (const auto& want : joint_names)
  {
    const auto it = std::find(src.name.begin(), src.name.end(), want);
    if (it == src.name.end())
    {
      missing.push_back(want);
      continue;
    }
    const auto i = static_cast<std::size_t>(std::distance(src.name.begin(), it));
    out.joint_state.name.push_back(want);
    if (i < src.position.size())
    {
      out.joint_state.position.push_back(src.position[i]);
    }
    // Velocity and effort are dropped on purpose: a waypoint is a posture, and
    // carrying a stale velocity into a trajectory start state is a way to get a
    // discontinuity nobody asked for.
  }

  if (!missing.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), fmt::format("input_state has no joint(s): {}",
                                                                            fmt::join(missing, ", ")));
    return BT::NodeStatus::FAILURE;
  }

  getBehaviorContext()->logger->publishInfoMessage(name(), fmt::format("kept {} of {} joints",
                                                                       out.joint_state.name.size(), src.name.size()));
  setOutput("output_state", out);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace joint_state_behaviors
