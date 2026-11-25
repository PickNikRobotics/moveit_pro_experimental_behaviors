// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <memory>
#include <string>

namespace yaml_behaviors
{

class SavePointStampedVectorToYaml final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SavePointStampedVectorToYaml(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace yaml_behaviors
