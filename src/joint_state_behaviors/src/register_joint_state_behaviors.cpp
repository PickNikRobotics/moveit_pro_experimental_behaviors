// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "joint_state_behaviors/register_joint_state_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "joint_state_behaviors/create_empty_multi_dof_joint_state.hpp"
#include "joint_state_behaviors/filter_robot_joint_state.hpp"
#include "joint_state_behaviors/filter_robot_state_to_joints.hpp"
#include "joint_state_behaviors/get_joint_value_from_robot_joint_state.hpp"
#include "joint_state_behaviors/is_joint_position_in_range.hpp"
#include "joint_state_behaviors/robot_state_from_pose_axis.hpp"
#include "joint_state_behaviors/set_joint_state_values.hpp"
#include "joint_state_behaviors/set_joint_value_in_robot_joint_state.hpp"

namespace joint_state_behaviors
{
void registerJointStateBehaviors(BT::BehaviorTreeFactory& factory,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<CreateEmptyMultiDOFJointState>(factory, "CreateEmptyMultiDOFJointState",
                                                                         shared_resources);
  moveit_pro::behaviors::registerBehavior<FilterRobotJointState>(factory, "FilterRobotJointState", shared_resources);
  moveit_pro::behaviors::registerBehavior<FilterRobotStateToJoints>(factory, "FilterRobotStateToJoints",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<GetJointValueFromRobotJointState>(factory, "GetJointValueFromRobotJointState",
                                                                            shared_resources);
  moveit_pro::behaviors::registerBehavior<IsJointPositionInRange>(factory, "IsJointPositionInRange", shared_resources);
  moveit_pro::behaviors::registerBehavior<RobotStateFromPoseAxis>(factory, "RobotStateFromPoseAxis", shared_resources);
  moveit_pro::behaviors::registerBehavior<SetJointStateValues>(factory, "SetJointStateValues");
  moveit_pro::behaviors::registerBehavior<SetJointValueInRobotJointState>(factory, "SetJointValueInRobotJointState",
                                                                          shared_resources);
}
}  // namespace joint_state_behaviors
