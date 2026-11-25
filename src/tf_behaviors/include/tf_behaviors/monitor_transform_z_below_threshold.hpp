#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace tf_behaviors
{

/// @brief Stateful monitor that returns SUCCESS the first tick `source_frame_id`'s Z coordinate
/// (in `target_frame_id`) is at or below `threshold_z`.
///
/// Put it in a Parallel with one required success, next to other stop conditions such as an F/T
/// monitor and a timeout, to stop a velocity-controlled descent at a height even if F/T never trips.
///
/// On TF lookup failure it returns RUNNING, not FAILURE, and retries on the next tick.
class MonitorTransformZBelowThreshold final : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  MonitorTransformZBelowThreshold(const std::string& name, const BT::NodeConfiguration& config,
                                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

}  // namespace tf_behaviors
