// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <control_msgs/msg/interface_value.hpp>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include "moveit_studio_behavior_interface/async_behavior_base.hpp"

namespace experimental_behaviors
{
/**
 * @brief Create a control_msgs::msg::InterfaceValue message and writes it to the Blackboard.
 *
 * @details
 * | Data Port Name       | Port Type   | Object Type                      |
 * | -------------------- | ----------- | -------------------------------- |
 * | interface_names     | input       | std::vector<std::string>         |
 * | values     | input       | std::vector<double>      |
 * | interface_value        | output      | control_msgs::msg::InterfaceValue |
 *
 * The Behavior checks to see if the input values `interface_names` and `values` are of the same and non-zero size.
 * # List of resource interface names
 */

class AccessInterfaceValueFromGroup final : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
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
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Returns list of ports for this behavior.
   * @return BT::PortsList
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /**
   * @brief Contains the logic to create a valid control_msgs::msg::InterfaceValue message.
   * @return BT::NodeStatus Returns success if successful, else failure.
   */
  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors