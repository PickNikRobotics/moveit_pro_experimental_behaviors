#include "planning_scene_behaviors/get_attached_collision_object_pose_stamped.hpp"

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <tl_expected/expected.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace
{
constexpr auto kPortIDCollisionObjectName = "collision_object_name";
constexpr auto kPortIDPlanningSceneMessage = "planning_scene";
constexpr auto kPortIDPoseStamped = "pose_stamped";
}  // namespace

namespace planning_scene_behaviors
{
GetAttachedCollisionObjectPoseStamped::GetAttachedCollisionObjectPoseStamped(const std::string& name,
                                                                             const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList GetAttachedCollisionObjectPoseStamped::providedPorts()
{
  return BT::PortsList({ BT::InputPort<std::string>(kPortIDCollisionObjectName, "{collision_object_name}",
                                                    "The name of the CollisionObject to get pose of"),
                         BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMessage, "{planning_scene}",
                                                                        "A planning scene"),
                         BT::OutputPort<geometry_msgs::msg::PoseStamped>(
                             kPortIDPoseStamped, "{pose_stamped}", "The pose of the attached collision object.") });
}

BT::KeyValueVector GetAttachedCollisionObjectPoseStamped::metadata()
{
  return { { "description", "Gets the pose of an attached collision object." },
           { "subcategory", "Perception - Planning Scene" } };
}

BT::NodeStatus GetAttachedCollisionObjectPoseStamped::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDCollisionObjectName),
                                               getInput<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMessage));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [collision_object_name, planning_scene] = ports.value();

  // Search attached objects for the requested id
  const auto& attached = planning_scene.robot_state.attached_collision_objects;
  const auto it = std::find_if(attached.begin(), attached.end(),
                               [&collision_object_name](const moveit_msgs::msg::AttachedCollisionObject& aco) {
                                 return aco.object.id == collision_object_name;
                               });

  if (it == attached.end())
  {
    spdlog::warn("GetAttachedCollisionObjectPoseStamped: no attached object named '{}'.", collision_object_name);
    return BT::NodeStatus::FAILURE;
  }

  const auto& obj = it->object;

  if (obj.header.frame_id.empty())
  {
    spdlog::warn("GetAttachedCollisionObjectPoseStamped: object '{}' has empty header.frame_id.", obj.id);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped out;
  out.header = obj.header;  // frame_id (and stamp, if set)
  out.pose = obj.pose;      // pose is already relative to header.frame_id

  if (!setOutput(kPortIDPoseStamped, out))
  {
    spdlog::warn("GetAttachedCollisionObjectPoseStamped: failed to set output port '{}'.", kPortIDPoseStamped);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace planning_scene_behaviors
