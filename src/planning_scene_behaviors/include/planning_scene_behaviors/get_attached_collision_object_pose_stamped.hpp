#pragma once

#include <behaviortree_cpp/action_node.h>

namespace planning_scene_behaviors
{
class GetAttachedCollisionObjectPoseStamped : public BT::SyncActionNode
{
public:
  GetAttachedCollisionObjectPoseStamped(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace planning_scene_behaviors
