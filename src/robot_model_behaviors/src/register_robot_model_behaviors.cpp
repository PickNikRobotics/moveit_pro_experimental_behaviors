// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "robot_model_behaviors/register_robot_model_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "robot_model_behaviors/get_joint_limits.hpp"
#include "robot_model_behaviors/trajectory_to_path.hpp"

namespace robot_model_behaviors
{
void registerRobotModelBehaviors(BT::BehaviorTreeFactory& factory,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<GetJointLimits>(factory, "GetJointLimits", shared_resources);
  moveit_pro::behaviors::registerBehavior<TrajectoryToPath>(factory, "TrajectoryToPath", shared_resources);
}
}  // namespace robot_model_behaviors
