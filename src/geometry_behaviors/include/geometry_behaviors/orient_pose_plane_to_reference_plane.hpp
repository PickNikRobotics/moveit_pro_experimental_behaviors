// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <string>

namespace geometry_behaviors
{
class OrientPosePlaneToReferencePlane : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  OrientPosePlaneToReferencePlane(const std::string& name, const BT::NodeConfiguration& config,
                                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace geometry_behaviors
