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
 * @brief Appends a value to a sequence at a nested key path in a YAML
 * file. Companion to `WriteYamlValue` (which sets) and `ReadYamlList`
 * (which reads). Reads don't mutate, so there is no read-side mirror
 * for "append" specifically.
 *
 * | Data Port Name | Port Type | Object Type  |
 * | -------------- | --------- | ------------ |
 * | file_path      | input     | std::string  |
 * | key1           | input     | std::string  |
 * | key2..key5     | input     | std::string  |
 * | value          | input     | std::string  |
 *
 * Same `key1`–`key5` port shape as `WriteYamlValue` / `ReadYamlList`.
 * The keyed location must either be absent (a new sequence is
 * created) or already be a sequence; appending into a scalar or map
 * fails the tick. The `value` port is parsed as YAML before being
 * appended, so scalars and inline maps both work.
 *
 * Round-trips through yaml-cpp on every tick: load existing → modify
 * → dump. Fails the tick if the file doesn't exist.
 */
class AppendYamlListItem final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  AppendYamlListItem(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
