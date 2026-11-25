#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace grasping_behaviors
{

class DeterminePickAlignment final : public BT::SyncActionNode
{
public:
  DeterminePickAlignment(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace grasping_behaviors
