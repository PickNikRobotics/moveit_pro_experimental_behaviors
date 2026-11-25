#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <string>

namespace planning_scene_behaviors
{
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

/**
 * @brief Fetches only the allowed collision matrix component of the current planning scene, via the
 * /get_planning_scene service.
 *
 * A fast input for Behaviors that only read the allowed collision matrix. It does not replace
 * GetCurrentPlanningScene: the octomap, world object geometry and robot state are never fetched.
 */
class GetPlanningSceneACMOnly final : public moveit_pro::behaviors::ServiceClientBehaviorBase<GetPlanningScene>
{
public:
  explicit GetPlanningSceneACMOnly(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  explicit GetPlanningSceneACMOnly(
      const std::string& name, const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
      std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<GetPlanningScene>> client_interface);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;

  tl::expected<GetPlanningScene::Request, std::string> createRequest() override;

  tl::expected<bool, std::string> processResponse(const GetPlanningScene::Response& response) override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace planning_scene_behaviors
