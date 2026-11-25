#include "planning_scene_behaviors/apply_planning_scene_diff.hpp"

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIDPlanningScene = "planning_scene";
constexpr auto kPortIDServiceName = "apply_planning_scene_service";

constexpr std::chrono::seconds kServiceResponseTimeout{ 15 };
}  // namespace

namespace planning_scene_behaviors
{

ApplyPlanningSceneDiff::ApplyPlanningSceneDiff(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<ApplyPlanningScene>(name, config, shared_resources)
{
}

ApplyPlanningSceneDiff::ApplyPlanningSceneDiff(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<ApplyPlanningScene>> client_interface)
  : ServiceClientBehaviorBase<ApplyPlanningScene>(name, config, shared_resources, std::move(client_interface))
{
}

BT::PortsList ApplyPlanningSceneDiff::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDServiceName, "/apply_planning_scene",
                               "Name of the service advertised by the MoveIt2 ApplyPlanningScene MoveGroup "
                               "capability."),
    BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningScene, "{inflated_scene}",
                                                   "Planning scene diff to apply (e.g. output of InflateOctomap)."),
  };
}

BT::KeyValueVector ApplyPlanningSceneDiff::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Planning Scene" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Applies a planning scene diff to MoveIt via the /apply_planning_scene service. "
             "Use this after InflateOctomap to push the inflated octomap into the planning scene." } };
}

tl::expected<std::chrono::duration<double>, std::string> ApplyPlanningSceneDiff::getResponseTimeout()
{
  return kServiceResponseTimeout;
}

tl::expected<std::string, std::string> ApplyPlanningSceneDiff::getServiceName()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDServiceName));
  if (!ports.has_value())
  {
    return tl::make_unexpected(ports.error());
  }
  return std::get<0>(ports.value());
}

tl::expected<ApplyPlanningScene::Request, std::string> ApplyPlanningSceneDiff::createRequest()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<moveit_msgs::msg::PlanningScene>(kPortIDPlanningScene));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required input [planning_scene]: " + ports.error());
  }

  const auto& [scene] = ports.value();

  ApplyPlanningScene::Request request;
  request.scene = scene;
  request.scene.is_diff = true;
  return request;
}

tl::expected<bool, std::string> ApplyPlanningSceneDiff::processResponse(const ApplyPlanningScene::Response& response)
{
  if (!response.success)
  {
    return tl::make_unexpected("ApplyPlanningScene service returned failure.");
  }
  return true;
}

}  // namespace planning_scene_behaviors

template class moveit_pro::behaviors::ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>;
template class moveit_pro::behaviors::ClientInterfaceBase<moveit_msgs::srv::ApplyPlanningScene>;
