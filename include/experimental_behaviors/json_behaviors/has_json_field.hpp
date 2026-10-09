// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Condition that checks whether a field exists at a JSON Pointer path in a JSON document.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | json           | input     | std::string |
 * | path           | input     | std::string |
 *
 * @details
 * Returns SUCCESS if the field exists, including a field whose value is null, and FAILURE if it does not.
 * Also returns FAILURE, with a logged message, on invalid JSON or an invalid path.
 */
class HasJsonField final : public moveit_pro::behaviors::SharedResourcesNode<BT::ConditionNode>
{
public:
  HasJsonField(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
