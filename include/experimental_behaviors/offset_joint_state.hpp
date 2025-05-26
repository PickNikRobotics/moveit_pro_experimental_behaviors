#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace experimental_behaviors
/**
 * @brief Action node that applies an offset to a joint state value.
 *
 * @details This node allows modifying a joint state by applying an offset value.
 *
 * | Data Port Name      | Port Type     | Object Type    |
 * |---------------------|---------------|----------------|
 * | joint_name          | Input         | std::string    |
 * | offset_value        | Input         | double         |
 * | joint_state_in      | Input         | sensor_msgs::msg::JointState |
 * | joint_state_out     | Output        | sensor_msgs::msg::JointState |
 */
{
class OffsetJointState : public BT::SyncActionNode
{
public:
  OffsetJointState(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace experimental_behaviors
