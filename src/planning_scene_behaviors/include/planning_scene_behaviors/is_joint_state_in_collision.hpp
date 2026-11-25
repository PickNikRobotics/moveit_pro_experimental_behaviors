#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <memory>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <string>

namespace planning_scene_behaviors
{
class IsJointStateInCollision : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  IsJointStateInCollision(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace planning_scene_behaviors
