#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <moveit_pro_behavior_interface/clock_interface.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace utility_behaviors
{
/**
 * @brief Measure the difference between an input timepoint and the current timepoint, and emit a log message which
 * states the time elapsed.
 *
 * @details
 * | Data Port Name | Port Type | Object Type                                        |
 * |----------------|-----------|----------------------------------------------------|
 * | timepoint      | input     | std::chrono::time_point<std::chrono::steady_clock> |
 * | message        | input     | std::string                                        |
 */
class StopwatchLog : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  StopwatchLog(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  StopwatchLog(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
               std::unique_ptr<moveit_pro::behaviors::ClockInterfaceBase> clock_interface);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  /** @brief Interface to a steady clock used to retrieve the current epoch time. */
  std::unique_ptr<moveit_pro::behaviors::ClockInterfaceBase> clock_interface_;
};
}  // namespace utility_behaviors
