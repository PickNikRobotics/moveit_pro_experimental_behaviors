// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/is_object_attached_example.hpp>

#include <spdlog/spdlog.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <tl_expected/expected.hpp>

#include <optional>
#include <string>

namespace
{
constexpr auto kPortIDPlanningSceneMessage = "planning_scene";
constexpr auto kPortObjectID = "object_id";
constexpr auto kPortLinkName = "link_name";
}  // namespace

namespace moveit_studio::behaviors
{

IsObjectAttachedToExample::IsObjectAttachedToExample(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList IsObjectAttachedToExample::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortObjectID, "{object_id}", "CollisionObject ID (name) to check"),
    BT::InputPort<std::string>(kPortLinkName, "{link_name}", "Robot link name to check"),
    BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMessage, "{planning_scene}",
                                                   "The planning scene"),
  };
}

BT::KeyValueVector IsObjectAttachedToExample::metadata()
{
  return { { "description", "Checks a PlanningScene to determine if the given CollisionObject name is attached to the "
                            "specified robot link."
                            "Returns SUCCESS if the object and link are attached; FAILURE otherwise." },
           { "subcategory", "Perception - Planning Scene" } };
}

BT::NodeStatus IsObjectAttachedToExample::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<std::string>(kPortObjectID), getInput<std::string>(kPortLinkName),
      getInput<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMessage));

  if (!ports.has_value())
  {
    spdlog::warn("IsObjectAttachedToExample: missing required input(s): {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [object_id, link_name, planning_scene] = ports.value();

  if (object_id.empty() || link_name.empty())
  {
    spdlog::warn("IsObjectAttachedToExample: inputs must be non-empty (object_id='{}', link_name='{}')", object_id, link_name);
    return BT::NodeStatus::FAILURE;
  }

  const auto& attached_collision_objects = planning_scene.robot_state.attached_collision_objects;

  // Search through all attached collision objects to see if their name and the link they are attached to match the behavior's inputs
  std::optional<std::string> attached_link;
  for (const auto& aco : attached_collision_objects)
  {
    if (aco.object.id == object_id)
    {
      // Once we find the collision object, save the link name it is attached to
      attached_link = aco.link_name;
      break;
    }
  }

  if (!attached_link)
  {
    spdlog::debug("IsObjectAttachedToExample: object '{}' is not attached to any link", object_id);
    return BT::NodeStatus::FAILURE;
  }

  // Check if the input link name matches the attached link name
  const bool match = (*attached_link == link_name);

  spdlog::debug("IsObjectAttachedToExample: object '{}' is attached to '{}' NOT '{}'", object_id, *attached_link, link_name);

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace moveit_studio::behaviors