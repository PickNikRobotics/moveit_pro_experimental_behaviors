// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Removes the field at a JSON Pointer path from a JSON document.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | json           | inout     | std::string |
 * | path           | input     | std::string |
 *
 * @details
 * Succeeds without changing the document if the field does not exist. Fails on invalid JSON, an invalid
 * path, or the root path (an empty string).
 */
class RemoveJsonField final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  RemoveJsonField(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
