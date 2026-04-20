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
 * @brief Reads a blackboard entry whose key is determined dynamically at tick
 *        time from an input port, and copies its value to an output port.
 *
 * @details
 * This is the read-side complement of BT.CPP's native SetBlackboard, which
 * already supports a dynamic `output_key` port but has no corresponding
 * read-by-dynamic-key primitive.
 *
 * | Data Port Name | Port Type | Object Type   |
 * | -------------- | --------- | ------------- |
 * | key            | input     | std::string   |
 * | value          | output    | any (BT::Any) |
 *
 * The `key` port accepts both literal strings and root-scope references
 * prefixed with `@`. Returns FAILURE if the key is missing or empty.
 */
class GetBlackboardByKey final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetBlackboardByKey(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
