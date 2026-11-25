// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <control_msgs/msg/interface_value.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include "moveit_pro_behavior_interface/async_behavior_base.hpp"

namespace control_interface_behaviors
{
/**
 * @brief Finds one interface by name in a control_msgs::msg::InterfaceValue message and outputs its value.
 *
 * @details
 * | Data Port Name          | Port Type | Object Type                       |
 * | ----------------------- | --------- | --------------------------------- |
 * | interface_name          | input     | std::string                       |
 * | dynamic_interface_value | input     | control_msgs::msg::InterfaceValue |
 * | value                   | output    | double                            |
 *
 * Registered as the Behavior ID `AccessInterfaceValue`. Returns FAILURE if the name is not in the message.
 */

class AccessInterfaceValueFromGroup final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Construct a new AccessInterfaceValueFromGroup behavior
   *
   * @param name See \ref SharedResourcesNode
   * @param config See \ref SharedResourcesNode
   * @param shared_resources See \ref SharedResourcesNode
   */
  AccessInterfaceValueFromGroup(const std::string& name, const BT::NodeConfiguration& config,
                                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Returns list of ports for this behavior.
   * @return BT::PortsList
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /**
   * @brief Finds the interface by name and writes its value to the output port.
   * @return BT::NodeStatus Returns success if successful, else failure.
   */
  BT::NodeStatus tick() override;
};
}  // namespace control_interface_behaviors
