#include "joint_state_behaviors/create_empty_multi_dof_joint_state.hpp"

namespace joint_state_behaviors
{

CreateEmptyMultiDOFJointState::CreateEmptyMultiDOFJointState(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CreateEmptyMultiDOFJointState::providedPorts()
{
  return { BT::OutputPort<sensor_msgs::msg::MultiDOFJointState>("output_state", "Empty MultiDOFJointState message") };
}

BT::KeyValueVector CreateEmptyMultiDOFJointState::metadata()
{
  return { { "description", "Creates an empty MultiDOFJointState message" }, { "subcategory", "Robot State" } };
}

BT::NodeStatus CreateEmptyMultiDOFJointState::tick()
{
  // Create an empty MultiDOFJointState message
  sensor_msgs::msg::MultiDOFJointState empty_state;

  // Set output
  setOutput("output_state", empty_state);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace joint_state_behaviors
