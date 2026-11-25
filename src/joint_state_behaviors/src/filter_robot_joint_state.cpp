#include "joint_state_behaviors/filter_robot_joint_state.hpp"

#include <algorithm>
#include <unordered_set>

#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>

namespace joint_state_behaviors
{

FilterRobotJointState::FilterRobotJointState(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList FilterRobotJointState::providedPorts()
{
  return { BT::InputPort<std::string>("planning_group_name", "Name of the planning group to filter joints for"),
           BT::InputPort<moveit_msgs::msg::RobotState>("input_robot_joint_state",
                                                       "Input robot joint state with all robot joints"),
           BT::OutputPort<moveit_msgs::msg::RobotState>(
               "output_robot_joint_state", "Output robot joint state containing only planning group joints") };
}

BT::KeyValueVector FilterRobotJointState::metadata()
{
  return { { "description", "Filters a JointState message to only include joints from a specific planning group" },
           { "subcategory", "Robot State" } };
}

tl::expected<bool, std::string> FilterRobotJointState::doWork()
{
  // Get input planning group name
  const auto planning_group_name = getInput<std::string>("planning_group_name");
  if (!planning_group_name)
  {
    return tl::make_unexpected("Failed to get required input [planning_group_name]: " + planning_group_name.error());
  }

  // Get input robot joint state
  const auto input_robot_joint_state = getInput<moveit_msgs::msg::RobotState>("input_robot_joint_state");
  if (!input_robot_joint_state)
  {
    return tl::make_unexpected("Failed to get required input [input_robot_joint_state]: " +
                               input_robot_joint_state.error());
  }

  // Extract the joint_state from the RobotState message
  const auto& input_joint_state = input_robot_joint_state->joint_state;

  // Get the planning group from the robot model
  const auto* planning_group = getBehaviorContext()->robot_model->getJointModelGroup(planning_group_name.value());
  if (!planning_group)
  {
    return tl::make_unexpected("Planning group '" + planning_group_name.value() + "' not found in robot model");
  }

  // Get joint names from the planning group
  const std::vector<std::string>& group_joint_names = planning_group->getVariableNames();

  // Create a set for fast lookup
  std::unordered_set<std::string> group_joint_set(group_joint_names.begin(), group_joint_names.end());

  // Create output robot joint state
  moveit_msgs::msg::RobotState output_robot_joint_state;
  output_robot_joint_state.joint_state.header = input_joint_state.header;
  // Copy multi_dof_joint_state as-is (not filtering it)
  output_robot_joint_state.multi_dof_joint_state = input_robot_joint_state->multi_dof_joint_state;

  // Pre-allocate output arrays
  output_robot_joint_state.joint_state.name.reserve(group_joint_names.size());
  const bool has_positions = !input_joint_state.position.empty();
  const bool has_velocities = !input_joint_state.velocity.empty();
  const bool has_efforts = !input_joint_state.effort.empty();

  if (has_positions)
  {
    output_robot_joint_state.joint_state.position.reserve(group_joint_names.size());
  }
  if (has_velocities)
  {
    output_robot_joint_state.joint_state.velocity.reserve(group_joint_names.size());
  }
  if (has_efforts)
  {
    output_robot_joint_state.joint_state.effort.reserve(group_joint_names.size());
  }

  // Filter joints - iterate through input and copy matching joints
  for (size_t i = 0; i < input_joint_state.name.size(); ++i)
  {
    const std::string& joint_name = input_joint_state.name[i];

    if (group_joint_set.find(joint_name) != group_joint_set.end())
    {
      output_robot_joint_state.joint_state.name.push_back(joint_name);

      if (has_positions && i < input_joint_state.position.size())
      {
        output_robot_joint_state.joint_state.position.push_back(input_joint_state.position[i]);
      }

      if (has_velocities && i < input_joint_state.velocity.size())
      {
        output_robot_joint_state.joint_state.velocity.push_back(input_joint_state.velocity[i]);
      }

      if (has_efforts && i < input_joint_state.effort.size())
      {
        output_robot_joint_state.joint_state.effort.push_back(input_joint_state.effort[i]);
      }
    }
  }

  // Validate that we found all expected joints
  if (output_robot_joint_state.joint_state.name.size() != group_joint_names.size())
  {
    return tl::make_unexpected("Planning group '" + planning_group_name.value() + "' has " +
                               std::to_string(group_joint_names.size()) + " joints, but only found " +
                               std::to_string(output_robot_joint_state.joint_state.name.size()) +
                               " matching joints in input joint state");
  }

  // Set output
  setOutput("output_robot_joint_state", output_robot_joint_state);

  return true;
}

}  // namespace joint_state_behaviors
