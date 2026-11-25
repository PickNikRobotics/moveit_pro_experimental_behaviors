// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <string>

namespace conversion_behaviors
{
class StringToDouble : public BT::SyncActionNode
{
public:
  StringToDouble(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace conversion_behaviors
