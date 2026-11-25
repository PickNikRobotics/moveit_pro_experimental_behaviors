// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace mtc_behaviors
{

/**
 * @brief Add an MTC stage that toggles ACM entries between two distinct groups of objects,
 *        without touching pairs WITHIN either group.
 *
 * @details Variant of the stock `SetupMTCIgnoreCollisionsBetweenObjects` that takes two lists. The stock
 * Behavior sets every pair of one list with itself, so a later disallow re-enables pairs the SRDF disables
 * as adjacent. See docs/AGENT_COMMENTS.md.
 *
 * This behavior calls `stage->allowCollisions(first_objects, second_objects, allow)` instead,
 * so only pairs `(a, b)` with `a` in `first_objects` and `b` in `second_objects` are touched.
 * Pairs within `first_objects` and pairs within `second_objects` are never visited, leaving
 * the SRDF defaults intact.
 *
 * Use this whenever you want to toggle the ACM between, say, a wrist-link group and a
 * surface group, without disturbing the wrist-internal pairs the SRDF disables for
 * mechanical adjacency.
 *
 * | Data Port Name  | Port Type     | Object Type                                 |
 * |-----------------|---------------|---------------------------------------------|
 * | first_objects   | input         | std::vector<std::string> (semicolon-sep'd)  |
 * | second_objects  | input         | std::vector<std::string> (semicolon-sep'd)  |
 * | allow_collision | input         | bool                                        |
 * | task            | bidirectional | moveit_pro::task_constructor::TaskPtr       |
 */
class SetCollisionsBetweenGroups final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SetCollisionsBetweenGroups(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  [[nodiscard]] static BT::PortsList providedPorts();

  [[nodiscard]] static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace mtc_behaviors
