// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <cstddef>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Select which trajectory point indices to keep when sparsifying a timed trajectory.
 *
 * @details Points in the leading window (`time_from_start <= start_preserve_seconds`) and the
 * trailing window (`time_from_start >= total_duration - end_preserve_seconds`) are always kept.
 * Points strictly between those windows (the "middle") are thinned to every `keep_every_nth`-th
 * point, counting from the first middle point so the first middle point is always kept.
 *
 * This is a pure function (deterministic, no side effects) extracted so it can be unit-tested
 * without any ROS or robot-model dependency.
 *
 * @param times Per-point `time_from_start` values in seconds. Assumed monotonically non-decreasing,
 *              which is the contract for a valid JointTrajectory.
 * @param start_preserve_seconds Leading duration to preserve verbatim. Negative values clamp to 0.
 * @param end_preserve_seconds Trailing duration to preserve verbatim. Negative values clamp to 0.
 * @param keep_every_nth Stride applied to the middle region. Values < 1 are treated as 1 (keep all).
 * @return Ascending, de-duplicated indices into `times`. Empty input yields an empty result. When
 *         the total duration is not positive (e.g. an untimed trajectory) the head/tail windows
 *         cannot be distinguished, so every index is kept.
 */
[[nodiscard]] std::vector<std::size_t> selectRetimeWaypointIndices(const std::vector<double>& times,
                                                                   double start_preserve_seconds,
                                                                   double end_preserve_seconds, int keep_every_nth);

/**
 * @brief Sparsifies a timed joint trajectory and re-times it with jerk-limited Ruckig smoothing.
 *
 * @details Built for the "speed up a slow recording" workflow: take a finely time-stamped joint
 * trajectory (e.g. from `RecordJointTrajectory`), keep the start and end exactly, thin out the dense
 * middle, then discard the original timing and regenerate a fresh parameterization. The output speed
 * is governed by the joint velocity/acceleration/jerk limits scaled by `velocity_scaling_factor`,
 * `acceleration_scaling_factor`, and `jerk_scale_factor` — not by the input timestamps, which are
 * thrown away. Use scaling factors near 1.0 for the fastest motion the limits allow.
 *
 * Ruckig smooths an already time-parameterized trajectory rather than generating timing from nothing,
 * so timing is first seeded with Time-Optimal Trajectory Generation (TOTG) and then refined by Ruckig
 * to respect jerk limits — the same TOTG-then-Ruckig pipeline used elsewhere in MoveIt Pro.
 *
 * Positions are reordered into the joint group's active-joint order before retiming, so the output is
 * correct even if the recording's `joint_names` are in a different order than the group.
 *
 * | Data Port Name              | Port Type | Object Type                            |
 * | --------------------------- | --------- | -------------------------------------- |
 * | joint_trajectory            | input     | trajectory_msgs::msg::JointTrajectory  |
 * | joint_group                 | input     | std::string                            |
 * | start_preserve_seconds      | input     | double                                 |
 * | end_preserve_seconds        | input     | double                                 |
 * | keep_every_nth              | input     | int                                    |
 * | velocity_scaling_factor     | input     | double                                 |
 * | acceleration_scaling_factor | input     | double                                 |
 * | jerk_scale_factor           | input     | double                                 |
 * | sampling_rate               | input     | int                                    |
 * | retimed_joint_trajectory    | output    | trajectory_msgs::msg::JointTrajectory  |
 */
class RetimeJointTrajectory final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  static constexpr auto kPortIDJointTrajectory = "joint_trajectory";
  static constexpr auto kPortIDJointGroup = "joint_group";
  static constexpr auto kPortIDStartPreserveSeconds = "start_preserve_seconds";
  static constexpr auto kPortIDEndPreserveSeconds = "end_preserve_seconds";
  static constexpr auto kPortIDKeepEveryNth = "keep_every_nth";
  static constexpr auto kPortIDVelocityScalingFactor = "velocity_scaling_factor";
  static constexpr auto kPortIDAccelerationScalingFactor = "acceleration_scaling_factor";
  static constexpr auto kPortIDJerkScaleFactor = "jerk_scale_factor";
  static constexpr auto kPortIDSamplingRate = "sampling_rate";
  static constexpr auto kPortIDRetimedJointTrajectory = "retimed_joint_trajectory";

  RetimeJointTrajectory(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors
