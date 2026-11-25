// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <tl_expected/expected.hpp>

namespace planning_scene_behaviors
{
using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;

/**
 * @brief Add a CollisionObject to the planning scene as a "virtual" obstacle with a caller-chosen
 *        RGBA color, in a single ApplyPlanningScene service call.
 *
 * @details Replaces a six-Behavior chain that ends in <code>AddVirtualObjectToPlanningScene</code>, and keeps
 *   its "virtual_" id prefix and allowed collision matrix rules, so objects from either Behavior coexist.
 *   See docs/AGENT_COMMENTS.md.
 *
 * | Data Port Name      | Port Type | Object Type                          |
 * | ------------------- | --------- | ------------------------------------ |
 * | collision_object    | Input     | moveit_msgs::msg::CollisionObject    |
 * | planning_scene_msg  | Input     | moveit_msgs::msg::PlanningScene      |
 * | r                   | Input     | double                               |
 * | g                   | Input     | double                               |
 * | b                   | Input     | double                               |
 * | a                   | Input     | double                               |
 */
class AddColoredVirtualObjectToPlanningScene final
  : public moveit_pro::behaviors::ServiceClientBehaviorBase<ApplyPlanningScene>
{
public:
  AddColoredVirtualObjectToPlanningScene(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /**
   * @brief Pure data transformation that builds the ApplyPlanningScene request.
   *
   * Public so unit tests can exercise the id-prefix logic, RGBA validation, and ACM
   * construction without standing up a BT tree or rclcpp service client. createRequest()
   * reads ports and delegates here.
   *
   * Preconditions: each of r, g, b, a is inside the unit interval, or it returns an error. The allowed
   * collision matrix must be square with rows as long as entry_names, as for AddVirtualObjectToPlanningScene;
   * an inconsistent snapshot gives a malformed result.
   */
  [[nodiscard]] static tl::expected<ApplyPlanningScene::Request, std::string>
  buildRequest(moveit_msgs::msg::CollisionObject collision_object, moveit_msgs::msg::PlanningScene planning_scene,
               double r, double g, double b, double a);

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<ApplyPlanningScene::Request, std::string> createRequest() override;

  tl::expected<bool, std::string> processResponse(const ApplyPlanningScene::Response& response) override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;

  // Captured in createRequest(), surfaced in processResponse() so the service-failure
  // message names the object that failed to apply.
  std::string last_request_id_;
};

}  // namespace planning_scene_behaviors
