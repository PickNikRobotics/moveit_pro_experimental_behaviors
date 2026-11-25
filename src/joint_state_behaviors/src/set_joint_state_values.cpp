// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "joint_state_behaviors/set_joint_state_values.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <algorithm>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr auto kPortIDJointStateIn = "joint_state_in";
constexpr auto kPortIDJointNames = "joint_names";
constexpr auto kPortIDJointValues = "joint_values";
constexpr auto kPortIDJointStateOut = "joint_state_out";
}  // namespace

namespace joint_state_behaviors
{

SetJointStateValues::SetJointStateValues(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetJointStateValues::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<sensor_msgs::msg::JointState>(kPortIDJointStateIn, "{joint_state_in}", "The JointState input"),
        BT::InputPort<std::vector<std::string>>(kPortIDJointNames, "{joint_names}", "The joints to overwrite"),
        BT::InputPort<std::vector<double>>(kPortIDJointValues, "{joint_values}", "The values to overwrite"),
        BT::OutputPort<sensor_msgs::msg::JointState>(kPortIDJointStateOut, "{joint_state_out}",
                                                     "The overwritten JointState") });
}

BT::KeyValueVector SetJointStateValues::metadata()
{
  return { { "description", "Overwrites joint values in a JointState message" },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus SetJointStateValues::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::JointState>(kPortIDJointStateIn),
                                               getInput<std::vector<std::string>>(kPortIDJointNames),
                                               getInput<std::vector<double>>(kPortIDJointValues));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [joint_state_in, joint_names, joint_values] = ports.value();

  if (joint_names.empty())
  {
    spdlog::warn("joint_names cannot be empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (joint_values.empty())
  {
    spdlog::warn("joint_values cannot be empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (joint_names.size() != joint_values.size())
  {
    spdlog::warn("joint_names and joint_values should be the same size.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& required_joint_names = joint_state_in.name;

  // Verify every element of joint_names is in required_joint_names
  bool all_found = std::ranges::all_of(joint_names, [&](const auto& name) {
    return std::ranges::find(required_joint_names, name) != required_joint_names.end();
  });

  if (!all_found)
  {
    spdlog::warn("Some elements of joint_names are not in the JointState message.");
    return BT::NodeStatus::FAILURE;
  }

  auto joint_state_out = joint_state_in;

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    const auto& name = joint_names[i];
    auto it = std::ranges::find(required_joint_names, name);
    if (it != required_joint_names.end())
    {
      std::size_t idx = std::distance(required_joint_names.begin(), it);
      joint_state_out.position[idx] = joint_values[i];
    }
  }

  setOutput(kPortIDJointStateOut, joint_state_out);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace joint_state_behaviors
