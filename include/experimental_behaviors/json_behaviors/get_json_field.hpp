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
 * @brief Reads the field at a JSON Pointer path in a JSON document.
 *
 * | Data Port Name | Port Type | Object Type          |
 * | -------------- | --------- | -------------------- |
 * | json           | input     | std::string          |
 * | path           | input     | std::string          |
 * | message_type   | input     | std::string          |
 * | value          | output    | any (AnyTypeAllowed) |
 *
 * @details
 * Without `message_type`, a string, boolean or number keeps its type (std::string, bool, int64_t, uint64_t
 * or double), so typed input ports downstream can read it. An object, array or null comes out as JSON text.
 *
 * With `message_type` (for example `geometry_msgs/msg/PoseStamped`), the field is converted to that ROS
 * message. Fails if the path does not exist, or if the field does not match the requested message type.
 */
class GetJsonField final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetJsonField(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
