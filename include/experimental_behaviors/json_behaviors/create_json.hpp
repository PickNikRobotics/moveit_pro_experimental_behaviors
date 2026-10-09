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
 * @brief Creates a JSON document on the blackboard, either an empty object or a validated copy of a template.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | initial        | input     | std::string |
 * | json           | output    | std::string |
 *
 * @details
 * `initial` defaults to `{}`. Any JSON value is accepted; the output is its compact serialization.
 * Fails if `initial` is not valid JSON, and the failure message gives the parse location.
 */
class CreateJson final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  CreateJson(const std::string& name, const BT::NodeConfiguration& config,
             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
