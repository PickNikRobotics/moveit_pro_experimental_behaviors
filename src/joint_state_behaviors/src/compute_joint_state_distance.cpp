// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "joint_state_behaviors/compute_joint_state_distance.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIDJointStateCurrent = "joint_state_current";
constexpr auto kPortIDJointStateTarget = "joint_state_target";
constexpr auto kPortIDJointNames = "joint_names";
constexpr auto kPortIDDistance = "distance";
constexpr auto kPortIDMaxJointDistance = "max_joint_distance";

std::unordered_map<std::string, double> positionsByName(const sensor_msgs::msg::JointState& state)
{
  std::unordered_map<std::string, double> positions;
  const std::size_t count = std::min(state.name.size(), state.position.size());
  for (std::size_t i = 0; i < count; ++i)
  {
    positions[state.name[i]] = state.position[i];
  }
  return positions;
}
}  // namespace

namespace joint_state_behaviors
{
namespace internal
{
tl::expected<JointStateDistance, std::string> jointStateDistance(const sensor_msgs::msg::JointState& current,
                                                                 const sensor_msgs::msg::JointState& target,
                                                                 const std::vector<std::string>& joint_names)
{
  std::vector<std::string> names;
  for (const auto& joint_name : joint_names.empty() ? target.name : joint_names)
  {
    if (!joint_name.empty())
    {
      names.push_back(joint_name);
    }
  }
  if (names.empty())
  {
    return tl::make_unexpected("there are no joints to compare: joint_names and the target state are both empty");
  }

  const auto current_positions = positionsByName(current);
  const auto target_positions = positionsByName(target);

  JointStateDistance result;
  double sum_of_squares = 0.0;
  for (const auto& joint_name : names)
  {
    const auto current_it = current_positions.find(joint_name);
    if (current_it == current_positions.end())
    {
      return tl::make_unexpected("joint '" + joint_name + "' has no position in joint_state_current");
    }
    const auto target_it = target_positions.find(joint_name);
    if (target_it == target_positions.end())
    {
      return tl::make_unexpected("joint '" + joint_name + "' has no position in joint_state_target");
    }
    const double difference = std::abs(current_it->second - target_it->second);
    if (!std::isfinite(difference))
    {
      return tl::make_unexpected("joint '" + joint_name + "' has a position that is not a finite number");
    }
    sum_of_squares += difference * difference;
    result.max_joint_distance = std::max(result.max_joint_distance, difference);
  }
  result.distance = std::sqrt(sum_of_squares);
  return result;
}
}  // namespace internal

ComputeJointStateDistance::ComputeJointStateDistance(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ComputeJointStateDistance::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::JointState>(kPortIDJointStateCurrent, "{joint_state_current}",
                                                       "First joint state, for example the current state."),
           BT::InputPort<sensor_msgs::msg::JointState>(kPortIDJointStateTarget, "{joint_state_target}",
                                                       "Second joint state, for example the goal."),
           BT::InputPort<std::vector<std::string>>(kPortIDJointNames, "",
                                                   "Joints to compare, separated by ';'. Empty means every joint named "
                                                   "in joint_state_target."),
           BT::OutputPort<double>(kPortIDDistance, "{joint_state_distance}",
                                  "Euclidean norm of the position differences, in the units of the joints."),
           BT::OutputPort<double>(kPortIDMaxJointDistance, "{max_joint_distance}",
                                  "Largest absolute position difference of one joint.") };
}

BT::KeyValueVector ComputeJointStateDistance::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Robot State" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Computes the joint-space distance between two joint states over the joints you name: the Euclidean "
             "norm of the position differences and the largest difference of one joint." } };
}

BT::NodeStatus ComputeJointStateDistance::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::JointState>(kPortIDJointStateCurrent),
                                               getInput<sensor_msgs::msg::JointState>(kPortIDJointStateTarget));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [current, target] = ports.value();
  const auto joint_names = getInput<std::vector<std::string>>(kPortIDJointNames).value_or(std::vector<std::string>{});

  const auto result = internal::jointStateDistance(current, target, joint_names);
  if (!result.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot compute the distance: " + result.error());
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDDistance, result->distance);
  setOutput(kPortIDMaxJointDistance, result->max_joint_distance);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace joint_state_behaviors
