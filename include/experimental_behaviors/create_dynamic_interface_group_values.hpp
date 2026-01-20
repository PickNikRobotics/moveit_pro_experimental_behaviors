// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <control_msgs/msg/dynamic_interface_group_values.hpp>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include "moveit_studio_behavior_interface/async_behavior_base.hpp"

namespace experimental_behaviors
{
/**
 * @brief Create a control_msgs::msg::DynamicInterfaceGroupValues message and writes it to the Blackboard.
 *
 * @details
 * | Data Port Name       | Port Type   | Object Type                      |
 * | -------------------- | ----------- | -------------------------------- |
 * | reference_frame      | input       | std::string                      |
 * | interface_groups     | input       | std::vector<std::string>         |
 * | interface_values     | input       | std::vector<control_msgs::msg::InterfaceValue>      |
 * | dynamic_interface_group_values        | output      | control_msgs::msg::DynamicInterfaceGroupValues |
 *
 * The Behavior checks to see if the input values `interface_groups` and `interface_values` are of the same and non-zero size.
 */

class CreateDynamicInterfaceGroupValues final : public SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Construct a new CreateDynamicInterfaceGroupValues behavior
   *
   * @param name See \ref SharedResourcesNode
   * @param config See \ref SharedResourcesNode
   * @param shared_resources See \ref SharedResourcesNode
   */
  CreateDynamicInterfaceGroupValues(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Returns list of ports for this behavior.
   * @return BT::PortsList
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /**
   * @brief Contains the logic to create a valid control_msgs::msg::DynamicInterfaceGroupValues message.
   * @return BT::NodeStatus Returns success if successful, else failure.
   */
  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
