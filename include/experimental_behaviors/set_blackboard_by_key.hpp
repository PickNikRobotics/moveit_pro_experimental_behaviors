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
 * @brief Writes the value of an input port to a blackboard entry whose key is
 *        determined dynamically at tick time.
 *
 * @details
 * Write-side complement to GetBlackboardByKey and a drop-in replacement for
 * BT.CPP's native SetBlackboard when the source value is carried on a port
 * typed as `AnyTypeAllowed` (e.g. a value produced by a Script `:=`
 * assignment).
 *
 * BT.CPP's SetBlackboard runs a string-conversion step whenever the
 * destination entry is not strictly `std::string` and the source value is a
 * string. For `AnyTypeAllowed` destinations there is no registered converter,
 * so the conversion produces an empty Any and silently clobbers the
 * destination — see set_blackboard_node.h and basic_types.cpp::parseString.
 *
 * This behavior copies the source Any directly, preserving both type and
 * value, which matches the semantics callers expect when caching arbitrary
 * subtree outputs under runtime-computed keys.
 *
 * | Data Port Name | Port Type | Object Type          |
 * | -------------- | --------- | -------------------- |
 * | key            | input     | std::string          |
 * | value          | input     | any (AnyTypeAllowed) |
 *
 * The `key` port accepts both literal strings and root-scope references
 * prefixed with `@`. Returns FAILURE if `key` is missing/empty or if the
 * `value` port refers to a blackboard entry that does not exist.
 */
class SetBlackboardByKey final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SetBlackboardByKey(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
