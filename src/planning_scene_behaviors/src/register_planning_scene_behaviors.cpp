// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "planning_scene_behaviors/register_planning_scene_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "planning_scene_behaviors/add_colored_virtual_object_to_planning_scene.hpp"
#include "planning_scene_behaviors/add_robot_joint_state_to_planning_scene_msg.hpp"
#include "planning_scene_behaviors/apply_planning_scene_diff.hpp"
#include "planning_scene_behaviors/generate_shape_urdf.hpp"
#include "planning_scene_behaviors/get_attached_collision_object_pose_stamped.hpp"
#include "planning_scene_behaviors/get_planning_scene_acm_only.hpp"
#include "planning_scene_behaviors/inflate_octomap.hpp"
#include "planning_scene_behaviors/is_joint_state_in_collision.hpp"

namespace planning_scene_behaviors
{
void registerPlanningSceneBehaviors(BT::BehaviorTreeFactory& factory,
                                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<AddColoredVirtualObjectToPlanningScene>(
      factory, "AddColoredVirtualObjectToPlanningScene", shared_resources);
  moveit_pro::behaviors::registerBehavior<AddRobotJointStateToPlanningSceneMsg>(
      factory, "AddRobotJointStateToPlanningSceneMsg", shared_resources);
  moveit_pro::behaviors::registerBehavior<ApplyPlanningSceneDiff>(factory, "ApplyPlanningSceneDiff", shared_resources);
  moveit_pro::behaviors::registerBehavior<GenerateShapeUrdf>(factory, "GenerateShapeUrdf");
  moveit_pro::behaviors::registerBehavior<GetAttachedCollisionObjectPoseStamped>(
      factory, "GetAttachedCollisionObjectPoseStamped");
  moveit_pro::behaviors::registerBehavior<GetPlanningSceneACMOnly>(factory, "GetPlanningSceneACMOnly", shared_resources);
  moveit_pro::behaviors::registerBehavior<InflateOctomap>(factory, "InflateOctomap", shared_resources);
  moveit_pro::behaviors::registerBehavior<IsJointStateInCollision>(factory, "IsJointStateInCollision", shared_resources);
}
}  // namespace planning_scene_behaviors
