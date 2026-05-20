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
 * @brief Sets a value at a nested key path in a YAML file. Mirror of
 * `ReadYamlValue`, with an absolute `file_path` instead of
 * `package_name` + `relative_file_path` (because writes target
 * runtime-generated locations rather than installed package shares).
 *
 * | Data Port Name | Port Type | Object Type  |
 * | -------------- | --------- | ------------ |
 * | file_path      | input     | std::string  |
 * | key1           | input     | std::string  |
 * | key2..key5     | input     | std::string  |
 * | value          | input     | std::string  |
 *
 * `key1` is mandatory; `key2`–`key5` are optional and skipped when
 * empty (matching `ReadYamlValue`). `value` is parsed as YAML before
 * being stored, so scalars (`"B"`), inline maps
 * (`"{ option: A, success: false }"`), and sequences (`"[a, b, c]"`)
 * all flow through the same string port.
 *
 * Round-trips through yaml-cpp on every tick: load existing → modify
 * → dump. Each call is atomic, so partial files survive crashes
 * mid-operation. Fails the tick if the file doesn't exist (this
 * behavior does not create files; pair with whatever step is
 * responsible for initialization).
 */
class WriteYamlValue final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  WriteYamlValue(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
