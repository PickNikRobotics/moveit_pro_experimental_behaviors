#include "planning_scene_behaviors/get_planning_scene_acm_only.hpp"

#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

namespace
{
inline constexpr auto kDescriptionGetPlanningSceneACMOnly = R"(
                <p>
                    Fetches only the allowed collision matrix component of the current planning scene via the
                    /get_planning_scene service, skipping the octomap, world object geometry, and robot state
                    entirely. Much faster than GetCurrentPlanningScene when the result will only be used as input to
                    SetCollisionsBetweenObjects or SetCollisionsBetweenObjectsInPlanningScene.
                </p>
                <p>
                    Do not use this ahead of motion planning: the returned planning scene does not contain the
                    octomap or world object geometry.
                </p>
            )";

constexpr auto kPortIDGetPlanningSceneServiceName = "get_planning_scene_service";
constexpr auto kPortIDPlanningSceneMessage = "planning_scene_msg";

constexpr std::chrono::seconds kServiceResponseTimeoutSeconds{ 15 };
}  // namespace

namespace planning_scene_behaviors
{
GetPlanningSceneACMOnly::GetPlanningSceneACMOnly(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<GetPlanningScene>(name, config, shared_resources)
{
}

GetPlanningSceneACMOnly::GetPlanningSceneACMOnly(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<GetPlanningScene>> client_interface)
  : moveit_pro::behaviors::ServiceClientBehaviorBase<GetPlanningScene>(name, config, shared_resources,
                                                                       std::move(client_interface))
{
}

BT::PortsList GetPlanningSceneACMOnly::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDGetPlanningSceneServiceName, "/get_planning_scene",
                               "Name of the service advertised by the MoveIt2 GetPlanningScene MoveGroup "
                               "capability."),
    BT::OutputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMessage, "{planning_scene}",
                                                    "The current planning scene, containing only the "
                                                    "allowed_collision_matrix field."),
  };
}

BT::KeyValueVector GetPlanningSceneACMOnly::metadata()
{
  return { { "subcategory", "Perception - Planning Scene" }, { "description", kDescriptionGetPlanningSceneACMOnly } };
}

tl::expected<std::string, std::string> GetPlanningSceneACMOnly::getServiceName()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDGetPlanningSceneServiceName));

  if (!ports.has_value())
  {
    return tl::make_unexpected(ports.error());
  }

  return std::get<0>(ports.value());
}

tl::expected<std::chrono::duration<double>, std::string> GetPlanningSceneACMOnly::getResponseTimeout()
{
  return kServiceResponseTimeoutSeconds;
}

tl::expected<GetPlanningScene::Request, std::string> GetPlanningSceneACMOnly::createRequest()
{
  GetPlanningScene::Request request;
  request.components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  return request;
}

tl::expected<bool, std::string> GetPlanningSceneACMOnly::processResponse(const GetPlanningScene::Response& response)
{
  if (!setOutput(kPortIDPlanningSceneMessage, response.scene))
  {
    return tl::make_unexpected(std::string("Failed to set output port '") + kPortIDPlanningSceneMessage + "'.");
  }

  return true;
}
}  // namespace planning_scene_behaviors

template class moveit_pro::behaviors::ServiceClientBehaviorBase<moveit_msgs::srv::GetPlanningScene>;
template class moveit_pro::behaviors::ClientInterfaceBase<moveit_msgs::srv::GetPlanningScene>;
