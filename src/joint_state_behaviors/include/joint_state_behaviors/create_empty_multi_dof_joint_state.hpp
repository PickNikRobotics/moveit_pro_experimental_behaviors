#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>

namespace joint_state_behaviors
{

/**
 * @brief Creates an empty MultiDOFJointState message.
 *
 * This behavior creates and outputs an empty sensor_msgs/msg/MultiDOFJointState message.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * output_state | output | sensor_msgs::msg::MultiDOFJointState
 */
class CreateEmptyMultiDOFJointState : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  CreateEmptyMultiDOFJointState(const std::string& name, const BT::NodeConfiguration& config,
                                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace joint_state_behaviors
