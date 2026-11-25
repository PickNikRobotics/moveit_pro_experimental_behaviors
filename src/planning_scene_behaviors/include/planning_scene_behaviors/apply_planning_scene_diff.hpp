#pragma once

#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>

namespace planning_scene_behaviors
{
using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;

/**
 * @brief Applies a planning scene diff to the MoveIt planning scene via the ApplyPlanningScene service.
 *
 * Intended to be used after InflateOctomap to push the inflated octomap back into MoveIt.
 *
 * Data Port Name               | Port Type | Object Type
 * ---------------------------- | --------- | -----------
 * planning_scene               | input     | moveit_msgs::msg::PlanningScene
 * apply_planning_scene_service | input     | std::string
 */
class ApplyPlanningSceneDiff : public moveit_pro::behaviors::ServiceClientBehaviorBase<ApplyPlanningScene>
{
public:
  ApplyPlanningSceneDiff(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  ApplyPlanningSceneDiff(
      const std::string& name, const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
      std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<ApplyPlanningScene>> client_interface);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;
  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;
  tl::expected<ApplyPlanningScene::Request, std::string> createRequest() override;
  tl::expected<bool, std::string> processResponse(const ApplyPlanningScene::Response& response) override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};

}  // namespace planning_scene_behaviors
