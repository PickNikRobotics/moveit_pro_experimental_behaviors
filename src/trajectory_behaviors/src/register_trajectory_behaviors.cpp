// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "trajectory_behaviors/register_trajectory_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "trajectory_behaviors/check_blend_deviation.hpp"
#include "trajectory_behaviors/filter_aba_joint_trajectory.hpp"

namespace trajectory_behaviors
{
void registerTrajectoryBehaviors(BT::BehaviorTreeFactory& factory,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<CheckBlendDeviation>(factory, "CheckBlendDeviation", shared_resources);
  moveit_pro::behaviors::registerBehavior<FilterABAJointTrajectory>(factory, "FilterABAJointTrajectory",
                                                                    shared_resources);
}
}  // namespace trajectory_behaviors
