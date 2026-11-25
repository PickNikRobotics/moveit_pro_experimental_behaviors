#include "joint_state_behaviors/get_joint_value_from_robot_joint_state.hpp"

namespace joint_state_behaviors
{

GetJointValueFromRobotJointState::GetJointValueFromRobotJointState(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetJointValueFromRobotJointState::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::RobotState>("robot_joint_state", "Input robot joint state"),
           BT::InputPort<std::string>("joint_name", "Name of the joint to query"),
           BT::OutputPort<double>("joint_value", "Value of the joint") };
}

BT::KeyValueVector GetJointValueFromRobotJointState::metadata()
{
  return { { "description", "Gets a joint value from a RobotState message" }, { "subcategory", "Robot State" } };
}

BT::NodeStatus GetJointValueFromRobotJointState::tick()
{
  // Get input robot joint state
  const auto robot_joint_state = getInput<moveit_msgs::msg::RobotState>("robot_joint_state");
  if (!robot_joint_state)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [robot_joint_state]: " +
                                                                 robot_joint_state.error());
    return BT::NodeStatus::FAILURE;
  }

  // Get joint name
  const auto joint_name = getInput<std::string>("joint_name");
  if (!joint_name)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [joint_name]: " +
                                                                 joint_name.error());
    return BT::NodeStatus::FAILURE;
  }

  // Search for the joint in joint_state.name array
  for (size_t i = 0; i < robot_joint_state->joint_state.name.size(); ++i)
  {
    if (robot_joint_state->joint_state.name[i] == joint_name.value())
    {
      // Check if position array has this index
      if (i >= robot_joint_state->joint_state.position.size())
      {
        getBehaviorContext()->logger->publishWarnMessage(name(), "Joint '" + joint_name.value() +
                                                                     "' found but has no position value");
        return BT::NodeStatus::FAILURE;
      }

      // Set output
      setOutput("joint_value", robot_joint_state->joint_state.position[i]);
      return BT::NodeStatus::SUCCESS;
    }
  }

  getBehaviorContext()->logger->publishWarnMessage(name(),
                                                   "Joint '" + joint_name.value() + "' not found in robot joint state");
  return BT::NodeStatus::FAILURE;
}

}  // namespace joint_state_behaviors
