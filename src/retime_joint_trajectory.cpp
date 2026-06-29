// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/retime_joint_trajectory.hpp>

#include <algorithm>
#include <iterator>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit_pro_base/robot_model/joint_model.hpp>
#include <moveit_pro_base/robot_model/joint_model_group.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <moveit_pro_base/robot_trajectory/robot_trajectory.hpp>
#include <moveit_pro_base/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <moveit_pro_base/trajectory_processing/trajectory_tools.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
// Default maximum jerk when joint_limits.yaml does not specify jerk bounds.
// Matches the default used by the BlendJointTrajectories behavior and the Ruckig smoothing plugin.
constexpr double kDefaultMaxJerk = 1000.0;  // rad/s^3

inline constexpr auto kDescriptionRetimeJointTrajectory = R"(
                <p>
                    Sparsifies a finely time-stamped <code>trajectory_msgs::msg::JointTrajectory</code> and re-times it
                    with jerk-limited Ruckig smoothing. Built to speed up a slow recording: the original timing is
                    discarded and a fresh parameterization is generated, bounded by the joint
                    velocity/acceleration/jerk limits scaled by <code>velocity_scaling_factor</code>,
                    <code>acceleration_scaling_factor</code>, and <code>jerk_scale_factor</code>. Use scaling factors
                    near 1.0 for the fastest motion the limits allow.
                </p>
                <p>
                    Ruckig smooths an already time-parameterized trajectory, so timing is first seeded with
                    Time-Optimal Trajectory Generation (TOTG) and then refined by Ruckig to respect jerk limits — the
                    same approach used elsewhere in MoveIt Pro to produce jerk-limited motion.
                </p>
                <p>
                    Points within <code>start_preserve_seconds</code> of the start and within
                    <code>end_preserve_seconds</code> of the end (measured by the input's <code>time_from_start</code>)
                    are kept verbatim. Points in between are thinned to every <code>keep_every_nth</code>-th point.
                    Positions are reordered into the joint group's active-joint order before retiming, so the result is
                    correct even when the recording's joint ordering differs from the group.
                </p>
            )";
}  // namespace

namespace experimental_behaviors
{
std::vector<std::size_t> selectRetimeWaypointIndices(const std::vector<double>& times,
                                                     double start_preserve_seconds, double end_preserve_seconds,
                                                     int keep_every_nth)
{
  std::vector<std::size_t> kept;
  if (times.empty())
  {
    return kept;
  }
  kept.reserve(times.size());

  const double start_preserve = std::max(0.0, start_preserve_seconds);
  const double end_preserve = std::max(0.0, end_preserve_seconds);
  const std::size_t stride = keep_every_nth < 1 ? std::size_t{ 1 } : static_cast<std::size_t>(keep_every_nth);

  // `times` is monotonically non-decreasing for a valid trajectory, so the last entry is the duration.
  const double total_duration = times.back();
  const double tail_threshold = total_duration - end_preserve;

  std::size_t middle_counter = 0;
  for (std::size_t i = 0; i < times.size(); ++i)
  {
    const double t = times[i];
    // Head and tail windows are always preserved. When total_duration <= 0 (untimed input) the tail
    // threshold is <= 0, so every point lands in the tail and nothing is dropped.
    if (t <= start_preserve || t >= tail_threshold)
    {
      kept.push_back(i);
      continue;
    }
    // Middle region: keep every stride-th point, counting from the first middle point.
    if (middle_counter % stride == 0)
    {
      kept.push_back(i);
    }
    ++middle_counter;
  }
  return kept;
}

RetimeJointTrajectory::RetimeJointTrajectory(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList RetimeJointTrajectory::providedPorts()
{
  return {
    BT::InputPort<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory, "{joint_trajectory}",
                                                         "Timed joint trajectory to sparsify and re-time."),
    BT::InputPort<std::string>(kPortIDJointGroup, "manipulator",
                               "Joint group whose velocity/acceleration/jerk limits bound the retiming."),
    BT::InputPort<double>(kPortIDStartPreserveSeconds, 0.0,
                          "Leading duration (s) of waypoints kept verbatim and never downsampled."),
    BT::InputPort<double>(kPortIDEndPreserveSeconds, 0.0,
                          "Trailing duration (s) of waypoints kept verbatim and never downsampled."),
    BT::InputPort<int>(kPortIDKeepEveryNth, 1,
                       "Stride for the middle region: keep every Nth waypoint (1 keeps all, 3 keeps ~1/3). Must be "
                       ">= 1."),
    BT::InputPort<double>(kPortIDVelocityScalingFactor, 1.0,
                          "Velocity scaling factor in (0, 1] applied to joint limits during retiming."),
    BT::InputPort<double>(kPortIDAccelerationScalingFactor, 1.0,
                          "Acceleration scaling factor in (0, 1] applied to joint limits during retiming."),
    BT::InputPort<double>(kPortIDJerkScaleFactor, 1.0,
                          "Jerk scaling factor in (0, 1] applied to joint jerk limits during Ruckig smoothing."),
    BT::InputPort<int>(kPortIDSamplingRate, 100, "Output trajectory sampling rate in Hz."),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(kPortIDRetimedJointTrajectory, "{retimed_joint_trajectory}",
                                                          "Sparsified, freshly re-timed joint trajectory."),
  };
}

BT::KeyValueVector RetimeJointTrajectory::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionRetimeJointTrajectory } };
}

BT::NodeStatus RetimeJointTrajectory::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory),
      getInput<std::string>(kPortIDJointGroup), getInput<double>(kPortIDStartPreserveSeconds),
      getInput<double>(kPortIDEndPreserveSeconds), getInput<int>(kPortIDKeepEveryNth),
      getInput<double>(kPortIDVelocityScalingFactor), getInput<double>(kPortIDAccelerationScalingFactor),
      getInput<double>(kPortIDJerkScaleFactor), getInput<int>(kPortIDSamplingRate));
  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [trajectory, joint_group_name, start_preserve_seconds, end_preserve_seconds, keep_every_nth, vel_scale,
               acc_scale, jerk_scale, sampling_rate] = ports.value();

  if (trajectory.joint_names.empty() || trajectory.points.size() < 2)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Input trajectory must have joint names and at least 2 points to re-time.");
    return BT::NodeStatus::FAILURE;
  }

  if (keep_every_nth < 1)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "keep_every_nth must be >= 1 (got " + std::to_string(keep_every_nth) + ").");
    return BT::NodeStatus::FAILURE;
  }

  if (vel_scale <= 0.0 || vel_scale > 1.0 || acc_scale <= 0.0 || acc_scale > 1.0 || jerk_scale <= 0.0 ||
      jerk_scale > 1.0)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "velocity_scaling_factor, acceleration_scaling_factor, and jerk_scale_factor must all be in (0, 1].");
    return BT::NodeStatus::FAILURE;
  }

  const auto& robot_model = shared_resources_->robot_model;
  if (!robot_model)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Robot model is not loaded on the BehaviorContext. RetimeJointTrajectory requires "
                "load_robot_model=true.");
    return BT::NodeStatus::FAILURE;
  }

  const auto* joint_model_group = robot_model->getJointModelGroup(joint_group_name);
  if (!joint_model_group || joint_model_group->getActiveJointModels().empty())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Joint group '" + joint_group_name + "' does not exist or has no active joints.");
    return BT::NodeStatus::FAILURE;
  }

  // Map every active joint in the group to its position index in the trajectory so we can build
  // waypoints in the group's joint order. A mismatched order would otherwise silently misassign joints.
  const auto& group_joint_names = joint_model_group->getActiveJointModelNames();
  std::vector<std::size_t> position_index;
  position_index.reserve(group_joint_names.size());
  for (const auto& group_joint_name : group_joint_names)
  {
    const auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), group_joint_name);
    if (it == trajectory.joint_names.end())
    {
      shared_resources_->logger->publishFailureMessage(
          name(), "Joint group '" + joint_group_name + "' joint '" + group_joint_name +
                      "' is not present in the trajectory's joint_names.");
      return BT::NodeStatus::FAILURE;
    }
    position_index.push_back(static_cast<std::size_t>(std::distance(trajectory.joint_names.begin(), it)));
  }

  // Collect per-point time_from_start (seconds) for the head/tail/middle partition.
  std::vector<double> times;
  times.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points)
  {
    times.push_back(static_cast<double>(point.time_from_start.sec) +
                    static_cast<double>(point.time_from_start.nanosec) * 1e-9);
  }

  if (times.back() <= 0.0)
  {
    shared_resources_->logger->publishWarnMessage(
        name(), "Input trajectory has no positive timing; head/tail windows cannot be distinguished, so all "
                "waypoints are kept (no downsampling) before retiming.");
  }

  const std::vector<std::size_t> kept_indices =
      selectRetimeWaypointIndices(times, start_preserve_seconds, end_preserve_seconds, keep_every_nth);

  // Build a RobotTrajectory of the sparsified waypoints, reordered into group-joint order. Seed the
  // state from default values so joints outside the group hold sensible values; velocities and
  // accelerations are zeroed because timing is regenerated below.
  moveit_pro::base::robot_trajectory::RobotTrajectory robot_trajectory(robot_model, joint_group_name);
  moveit_pro::base::RobotState state(robot_model);
  state.setToDefaultValues();
  std::vector<double> ordered_positions(group_joint_names.size());
  for (const std::size_t index : kept_indices)
  {
    const auto& point = trajectory.points[index];
    if (point.positions.size() != trajectory.joint_names.size())
    {
      shared_resources_->logger->publishFailureMessage(
          name(), "Trajectory point " + std::to_string(index) + " has " + std::to_string(point.positions.size()) +
                      " positions but " + std::to_string(trajectory.joint_names.size()) + " joint names.");
      return BT::NodeStatus::FAILURE;
    }
    for (std::size_t j = 0; j < group_joint_names.size(); ++j)
    {
      ordered_positions[j] = point.positions[position_index[j]];
    }
    state.setVariablePositions(group_joint_names, ordered_positions);
    state.zeroVelocities();
    state.zeroAccelerations();
    state.update();
    // Seed duration is 0.0; TOTG recomputes timing from the path below.
    robot_trajectory.addSuffixWayPoint(state, 0.0);
  }

  // Seed timing with TOTG. Ruckig smoothing requires an already time-parameterized trajectory.
  if (!moveit_pro::base::trajectory_processing::applyTOTGTimeParameterization(robot_trajectory, vel_scale, acc_scale))
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Failed to seed trajectory timing with Time-Optimal Trajectory Generation.");
    return BT::NodeStatus::FAILURE;
  }

  // Build per-joint limit maps for Ruckig. Velocity/acceleration limits are passed unscaled (the scale
  // factors are applied by applySmoothing); jerk has no scale-factor argument, so bake jerk_scale in.
  std::unordered_map<std::string, double> velocity_limits;
  std::unordered_map<std::string, double> acceleration_limits;
  std::unordered_map<std::string, double> jerk_limits;
  for (const auto& group_joint_name : group_joint_names)
  {
    const moveit_pro::base::VariableBounds bounds =
        robot_model->getJointModel(group_joint_name)->getVariableBounds(group_joint_name);
    if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_)
    {
      shared_resources_->logger->publishFailureMessage(
          name(), "Joint '" + group_joint_name +
                      "' is missing velocity or acceleration bounds. Specify them in joint_limits.yaml.");
      return BT::NodeStatus::FAILURE;
    }
    velocity_limits[group_joint_name] = bounds.max_velocity_;
    acceleration_limits[group_joint_name] = bounds.max_acceleration_;
    jerk_limits[group_joint_name] = (bounds.jerk_bounded_ ? bounds.max_jerk_ : kDefaultMaxJerk) * jerk_scale;
  }

  // Refine the TOTG timing with jerk-limited Ruckig smoothing.
  if (!moveit_pro::base::trajectory_processing::RuckigSmoothing::applySmoothing(
          robot_trajectory, velocity_limits, acceleration_limits, jerk_limits, vel_scale, acc_scale))
  {
    shared_resources_->logger->publishFailureMessage(name(), "Ruckig smoothing failed to re-time the trajectory.");
    return BT::NodeStatus::FAILURE;
  }

  // Extract the re-timed RobotTrajectory back into a JointTrajectory message in group-joint order.
  std::vector<int> group_variable_indices;
  group_variable_indices.reserve(group_joint_names.size());
  for (const auto& group_joint_name : group_joint_names)
  {
    group_variable_indices.push_back(static_cast<int>(robot_model->getVariableIndex(group_joint_name)));
  }

  trajectory_msgs::msg::JointTrajectory retimed;
  retimed.header = trajectory.header;
  retimed.joint_names = group_joint_names;
  retimed.points.reserve(robot_trajectory.getWayPointCount());
  for (std::size_t i = 0; i < robot_trajectory.getWayPointCount(); ++i)
  {
    const auto& waypoint = robot_trajectory.getWayPoint(i);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(group_joint_names.size());
    point.velocities.resize(group_joint_names.size());
    point.accelerations.resize(group_joint_names.size());
    for (std::size_t j = 0; j < group_joint_names.size(); ++j)
    {
      const int variable_index = group_variable_indices[j];
      point.positions[j] = waypoint.getVariablePosition(variable_index);
      point.velocities[j] = waypoint.getVariableVelocity(variable_index);
      point.accelerations[j] = waypoint.getVariableAcceleration(variable_index);
    }
    const double time_from_start = robot_trajectory.getWayPointDurationFromStart(i);
    point.time_from_start.sec = static_cast<int32_t>(time_from_start);
    point.time_from_start.nanosec = static_cast<uint32_t>((time_from_start - static_cast<double>(point.time_from_start.sec)) * 1e9);
    retimed.points.push_back(std::move(point));
  }

  setOutput(kPortIDRetimedJointTrajectory, retimed);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
