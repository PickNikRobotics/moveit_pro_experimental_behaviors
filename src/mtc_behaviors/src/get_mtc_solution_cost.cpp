#include "mtc_behaviors/get_mtc_solution_cost.hpp"

#include <vector>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescription = R"(
<p>
Extracts the cost of an MTC solution. Reports the total cost, summed across all sub-trajectories, and the individual
cost of each sub-trajectory.
</p>
)";

constexpr auto kPortInSolution = "solution";

constexpr auto kPortOutTotalCost = "total_cost";
constexpr auto kPortOutSubTrajectoryCosts = "sub_trajectory_costs";
}  // namespace

namespace mtc_behaviors
{
GetMtcSolutionCost::GetMtcSolutionCost(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetMtcSolutionCost::providedPorts()
{
  return {
    BT::InputPort<moveit_task_constructor_msgs::msg::Solution>(kPortInSolution, "{mtc_solution}",
                                                               "MoveIt Task Constructor solution."),
    BT::OutputPort<double>(kPortOutTotalCost, "{total_cost}",
                           "Total cost of the solution, summed across all sub-trajectories."),
    BT::OutputPort<std::vector<double>>(kPortOutSubTrajectoryCosts, "{sub_trajectory_costs}",
                                        "Cost of each individual sub-trajectory, in order."),
  };
}

BT::KeyValueVector GetMtcSolutionCost::metadata()
{
  return {
    { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Task Planning" },
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
  };
}

BT::NodeStatus GetMtcSolutionCost::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<moveit_task_constructor_msgs::msg::Solution>(kPortInSolution));

  if (!ports)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required input ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [solution] = ports.value();

  double total_cost = 0.0;
  std::vector<double> sub_trajectory_costs;
  sub_trajectory_costs.reserve(solution.sub_trajectory.size());

  for (const auto& sub_traj : solution.sub_trajectory)
  {
    const double cost = static_cast<double>(sub_traj.info.cost);
    sub_trajectory_costs.push_back(cost);
    total_cost += cost;
  }

  setOutput(kPortOutTotalCost, total_cost);
  setOutput(kPortOutSubTrajectoryCosts, sub_trajectory_costs);

  spdlog::info("MTC solution total cost: {} ({} sub-trajectories).", total_cost, sub_trajectory_costs.size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_behaviors
