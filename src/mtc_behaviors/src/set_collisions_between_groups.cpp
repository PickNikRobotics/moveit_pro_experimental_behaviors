// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "mtc_behaviors/set_collisions_between_groups.hpp"

#include <sstream>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro/task_constructor/stages.h>
#include <moveit_pro/task_constructor/task.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kDescription = R"(
                <p>Add an MTC Stage, named '<allow/forbid> collision (group vs. group)', to an MTC Task that modifies the planning scene's Allowed Collision Matrix to permit or forbid collisions between every pair (a, b) where a is drawn from `first_objects` and b is drawn from `second_objects`. Pairs within `first_objects` (e.g. wrist-link to wrist-link) and pairs within `second_objects` (e.g. scene object to scene object) are NOT touched, so SRDF `<disable_collisions reason="Adjacent">` defaults are preserved.</p>
                <p>Compared to `SetupMTCIgnoreCollisionsBetweenObjects` (single list, cross product with itself), this behavior is the safe choice when one of the groups contains mechanically-adjacent links the SRDF wants always-disabled — typical for the wrist chain (gripper body, tool adapter and camera adapter links, etc.) where adjacent pairs would otherwise get clobbered by the disallow call.</p>
                <p>This Behavior will add stage(s) that will modify the global planning scene when executed via the `Execute MTC Solution` Subtree.</p>
            )";

constexpr auto kPortIDTask = "task";
constexpr auto kPortIDFirstObjects = "first_objects";
constexpr auto kPortIDSecondObjects = "second_objects";
constexpr auto kPortIDAllowCollision = "allow_collision";
}  // namespace

namespace mtc_behaviors
{
SetCollisionsBetweenGroups::SetCollisionsBetweenGroups(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetCollisionsBetweenGroups::providedPorts()
{
  return { BT::InputPort<std::vector<std::string>>(kPortIDFirstObjects, "",
                                                   "Semicolon-separated list of object names — group A. ACM "
                                                   "entries are toggled between every member of this group and "
                                                   "every member of `second_objects`. Pairs within this group "
                                                   "are NOT touched."),
           BT::InputPort<std::vector<std::string>>(kPortIDSecondObjects, "",
                                                   "Semicolon-separated list of object names — group B. ACM "
                                                   "entries are toggled between every member of this group and "
                                                   "every member of `first_objects`. Pairs within this group "
                                                   "are NOT touched."),
           BT::InputPort<bool>(kPortIDAllowCollision, true,
                               "If true, allow collisions between every (a, b) pair where a is in `first_objects` "
                               "and b is in `second_objects`. If false, forbid them."),
           BT::BidirectionalPort<moveit_pro::task_constructor::TaskPtr>(kPortIDTask, "{mtc_task}",
                                                                        "MoveIt Task Constructor task.") };
}

BT::KeyValueVector SetCollisionsBetweenGroups::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Task Planning" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus SetCollisionsBetweenGroups::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<moveit_pro::task_constructor::TaskPtr>(kPortIDTask),
                                               getInput<std::vector<std::string>>(kPortIDFirstObjects),
                                               getInput<std::vector<std::string>>(kPortIDSecondObjects),
                                               getInput<bool>(kPortIDAllowCollision));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [task, first_objects, second_objects, allow_collision] = ports.value();

  if (first_objects.empty() || second_objects.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Both `" + std::string(kPortIDFirstObjects) +
                                                                    "` and `" + std::string(kPortIDSecondObjects) +
                                                                    "` must be non-empty lists of object names.");
    return BT::NodeStatus::FAILURE;
  }

  std::stringstream stage_name;
  stage_name << (allow_collision ? "allow" : "forbid") << " collision (group vs. group)";

  auto stage =
      std::make_unique<moveit_pro::task_constructor::stages::ModifyPlanningScene>(std::string(stage_name.str()));
  stage->allowCollisions(first_objects, second_objects, allow_collision);
  task->insert(std::move(stage));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_behaviors
