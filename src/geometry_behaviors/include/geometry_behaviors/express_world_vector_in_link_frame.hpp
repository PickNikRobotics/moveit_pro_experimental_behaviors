#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace geometry_behaviors
{

/// @brief Re-expresses a 3D vector given in world frame as the equivalent vector in a link's local
/// frame, using that link's current world orientation (a quaternion).
///
/// Computes `local = q_link^{-1} * world`, for example to give the velocity force controller a
/// world-frame direction in its end-effector frame. See docs/AGENT_COMMENTS.md.
class ExpressWorldVectorInLinkFrame final : public BT::SyncActionNode
{
public:
  ExpressWorldVectorInLinkFrame(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace geometry_behaviors
