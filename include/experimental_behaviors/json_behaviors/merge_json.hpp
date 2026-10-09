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
 * @brief Applies a JSON Merge Patch (RFC 7386) to a JSON document.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | json           | inout     | std::string |
 * | patch          | input     | std::string |
 *
 * @details
 * Objects in `patch` merge recursively into `json`. A null in `patch` removes that key. Any other value,
 * including an array, replaces the target. Fails if either input is not valid JSON.
 */
class MergeJson final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  MergeJson(const std::string& name, const BT::NodeConfiguration& config,
            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
