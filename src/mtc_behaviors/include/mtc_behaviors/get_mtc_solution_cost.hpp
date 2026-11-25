#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

namespace mtc_behaviors
{
/**
 * @brief Extracts the cost of a MoveIt Task Constructor Solution.
 *
 * @details
 * | Data Port Name           | Port Type     | Object Type                                 |
 * | ------------------------ |--------------|----------------------------------------------|
 * | solution                 | Input         | moveit_task_constructor_msgs::msg::Solution  |
 * | total_cost               | Output        | double                                       |
 * | sub_trajectory_costs     | Output        | std::vector<double>                          |
 */
class GetMtcSolutionCost final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetMtcSolutionCost(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
};
}  // namespace mtc_behaviors
