// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace moveit_studio::behaviors
{

class IsObjectAttachedToExample : public BT::SyncActionNode
{
public:
  IsObjectAttachedToExample(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace moveit_studio::behaviors