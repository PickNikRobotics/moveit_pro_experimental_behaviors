// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <control_msgs/msg/interface_value.hpp>

#include <experimental_behaviors/shared_resources_node.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include "moveit_pro_behavior_interface/async_behavior_base.hpp"

namespace experimental_behaviors
{
/**
 * @brief Read a named scalar from a control_msgs::msg::InterfaceValue message.
 *
 * @details
 * | Data Port Name | Port Type | Object Type |
 * | --- | --- | --- |
 * | interface_name | input | std::string |
 * | dynamic_interface_value | input | control_msgs::msg::InterfaceValue |
 * | value | output | double |
 */

class AccessInterfaceValueFromGroup final : public ExperimentalSharedResourcesNode<BT::SyncActionNode>
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
   * @brief Read the requested scalar interface.
   * @return BT::NodeStatus Returns success if successful, else failure.
   */
  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
