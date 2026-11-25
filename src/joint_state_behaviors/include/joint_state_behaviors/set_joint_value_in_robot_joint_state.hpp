#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace joint_state_behaviors
{

/**
 * @brief Sets a joint value in a RobotState message.
 *
 * This behavior takes a RobotState, a joint name, and a new value,
 * and returns a modified RobotState with the updated joint value.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * input_robot_joint_state | input | moveit_msgs::msg::RobotState
 * joint_name | input | std::string
 * joint_value | input | double
 * output_robot_joint_state | output | moveit_msgs::msg::RobotState
 */
class SetJointValueInRobotJointState : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SetJointValueInRobotJointState(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace joint_state_behaviors
