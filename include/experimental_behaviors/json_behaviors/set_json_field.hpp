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
 * @brief Adds or replaces the field at a JSON Pointer path in a JSON document.
 *
 * | Data Port Name      | Port Type | Object Type          |
 * | ------------------- | --------- | -------------------- |
 * | json                | inout     | std::string          |
 * | path                | input     | std::string          |
 * | value               | input     | any (AnyTypeAllowed) |
 * | create_missing      | input     | bool                 |
 * | parse_value_as_json | input     | bool                 |
 *
 * @details
 * Strings, booleans and numbers on the blackboard map to the matching JSON type. Other types, such as ROS
 * messages, go through the JSON converters registered with BT::JsonExporter, so the result matches the
 * UI blackboard view. A literal value written in the XML is a JSON string unless `parse_value_as_json` is true.
 *
 * With `create_missing` (default true), missing parents are created as objects. An array index equal to
 * the array size, or `-`, appends. Fails on invalid JSON, an invalid path, a parent that cannot hold
 * fields, or a value that cannot be serialized (the message names its C++ type).
 */
class SetJsonField final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SetJsonField(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
