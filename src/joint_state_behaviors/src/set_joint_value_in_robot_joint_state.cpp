#include "joint_state_behaviors/set_joint_value_in_robot_joint_state.hpp"

namespace joint_state_behaviors
{

SetJointValueInRobotJointState::SetJointValueInRobotJointState(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetJointValueInRobotJointState::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::RobotState>("input_robot_joint_state", "Input robot joint state"),
           BT::InputPort<std::string>("joint_name", "Name of the joint to modify"),
           BT::InputPort<double>("joint_value", "New value for the joint"),
           BT::OutputPort<moveit_msgs::msg::RobotState>("output_robot_joint_state",
                                                        "Output robot joint state with modified value") };
}

BT::KeyValueVector SetJointValueInRobotJointState::metadata()
{
  return { { "description", "Sets a joint value in a RobotState message" }, { "subcategory", "Robot State" } };
}

BT::NodeStatus SetJointValueInRobotJointState::tick()
{
  // Get input robot joint state
  const auto input_robot_joint_state = getInput<moveit_msgs::msg::RobotState>("input_robot_joint_state");
  if (!input_robot_joint_state)
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), "Failed to get required input [input_robot_joint_state]: " + input_robot_joint_state.error());
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

  // Get joint value
  const auto joint_value = getInput<double>("joint_value");
  if (!joint_value)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [joint_value]: " +
                                                                 joint_value.error());
    return BT::NodeStatus::FAILURE;
  }

  // Create output by copying input
  moveit_msgs::msg::RobotState output_robot_joint_state = input_robot_joint_state.value();

  // Search for the joint in joint_state.name array
  bool joint_found = false;
  for (size_t i = 0; i < output_robot_joint_state.joint_state.name.size(); ++i)
  {
    if (output_robot_joint_state.joint_state.name[i] == joint_name.value())
    {
      // Ensure position array is large enough
      if (i >= output_robot_joint_state.joint_state.position.size())
      {
        output_robot_joint_state.joint_state.position.resize(i + 1, 0.0);
      }
      output_robot_joint_state.joint_state.position[i] = joint_value.value();
      joint_found = true;
      break;
    }
  }

  if (!joint_found)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Joint '" + joint_name.value() +
                                                                 "' not found in robot joint state");
    return BT::NodeStatus::FAILURE;
  }

  // Set output
  setOutput("output_robot_joint_state", output_robot_joint_state);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace joint_state_behaviors
