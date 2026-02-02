// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <control_msgs/msg/interface_value.hpp>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include "moveit_studio_behavior_interface/async_behavior_base.hpp"

namespace experimental_behaviors
{
/**
 * @brief Get an InterfaceValue from a DynamicInterfaceGroupValues message by group name.
 *
 * @details
 * | Data Port Name                    | Port Type   | Object Type                                    |
 * | --------------------------------- | ----------- | ---------------------------------------------- |
 * | group_values                      | input       | control_msgs::msg::DynamicInterfaceGroupValues |
 * | interface_group_name              | input       | std::string                                    |
 * | interface_value                   | output      | control_msgs::msg::InterfaceValue              |
 *
 * The Behavior searches for the interface group by name and returns the corresponding InterfaceValue.
 * Returns FAILURE if the group name is not found in the DynamicInterfaceGroupValues message.
 */

class GetInterfaceValueFromGroup final : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Construct a new GetInterfaceValueFromGroup behavior
   *
   * @param name See \ref SharedResourcesNode
   * @param config See \ref SharedResourcesNode
   * @param shared_resources See \ref SharedResourcesNode
   */
  GetInterfaceValueFromGroup(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Returns list of ports for this behavior.
   * @return BT::PortsList
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  /**
   * @brief Implementation of the behavior.
   * @return BT::NodeStatus
   */
  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
