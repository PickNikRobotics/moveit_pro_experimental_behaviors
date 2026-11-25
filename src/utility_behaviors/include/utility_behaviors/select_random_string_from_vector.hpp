#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace utility_behaviors
{
/**
 * @brief Selects a random element from an input vector of strings.
 */
class SelectRandomStringFromVector : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SelectRandomStringFromVector(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace utility_behaviors
