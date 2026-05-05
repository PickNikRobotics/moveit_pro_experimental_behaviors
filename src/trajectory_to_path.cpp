// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/trajectory_to_path.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
inline constexpr auto kDescriptionTrajectoryToPath = R"(
                <p>
                    Converts a <code>trajectory_msgs::msg::JointTrajectory</code> into a list of stamped Cartesian
                    poses by running forward kinematics on a chosen tip link, one pose per trajectory point.
                </p>
                <p>
                    The output port <code>path</code> is shaped to plug directly into <code>VisualizePath</code>'s
                    <code>path</code> input, allowing a joint-space trajectory to be visualized before being passed
                    to <code>ExecuteTrajectory</code>. Joints absent from the trajectory keep their current planning
                    scene values during FK.
                </p>
            )";
}  // namespace

namespace experimental_behaviors
{
TrajectoryToPath::TrajectoryToPath(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList TrajectoryToPath::providedPorts()
{
  return {
    BT::InputPort<trajectory_msgs::msg::JointTrajectory>(
        kPortIDJointTrajectory, "{joint_trajectory}",
        "Joint-space trajectory to convert. Same type ExecuteTrajectory consumes."),
    BT::InputPort<std::string>(
        kPortIDTipLink, "{tip_link}",
        "Robot link whose Cartesian pose is reported once per trajectory point."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        kPortIDPath, "{path}",
        "One stamped pose per trajectory point, in the robot model's root frame. "
        "Plugs directly into VisualizePath's `path` input port."),
  };
}

BT::KeyValueVector TrajectoryToPath::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Visualization" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionTrajectoryToPath } };
}

BT::NodeStatus TrajectoryToPath::tick()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  if (!getInput<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory, trajectory))
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing required input port [joint_trajectory].");
    return BT::NodeStatus::FAILURE;
  }

  std::string tip_link;
  if (!getInput<std::string>(kPortIDTipLink, tip_link) || tip_link.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing or empty required input port [tip_link].");
    return BT::NodeStatus::FAILURE;
  }

  const auto& robot_model = shared_resources_->robot_model;
  if (!robot_model)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Robot model is not loaded on the BehaviorContext. TrajectoryToPath requires "
                "load_robot_model=true.");
    return BT::NodeStatus::FAILURE;
  }

  // Validate the tip link.
  if (!robot_model->hasLinkModel(tip_link))
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     "Unknown tip_link '" + tip_link + "' (not in robot model).");
    return BT::NodeStatus::FAILURE;
  }

  // Validate every joint name in the trajectory.
  for (const auto& joint_name : trajectory.joint_names)
  {
    if (!robot_model->hasJointModel(joint_name))
    {
      shared_resources_->logger->publishFailureMessage(
          name(), "Trajectory contains joint '" + joint_name + "' which is not in the robot model.");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Seed FK from the planning scene's current state (not URDF defaults) so joints
  // absent from the trajectory — e.g. the rail or the other arm in dual-arm setups —
  // keep their actual current values rather than snapping to neutral.
  moveit_pro::base::planning_scene::PlanningScene planning_scene(robot_model);
  const moveit_pro::base::RobotState base_state = planning_scene.getCurrentState();

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(trajectory.points.size());

  const auto stamp = shared_resources_->node->get_clock()->now();
  const auto& frame_id = robot_model->getModelFrame();

  // Reuse a single RobotState across iterations: setVariablePositions only touches
  // the variables in joint_names, so non-trajectory joints retain their seeded values
  // and we avoid a per-point deep copy of the full state.
  moveit_pro::base::RobotState state = base_state;

  for (const auto& point : trajectory.points)
  {
    if (point.positions.size() != trajectory.joint_names.size())
    {
      shared_resources_->logger->publishFailureMessage(
          name(), "Trajectory point has " + std::to_string(point.positions.size()) +
                      " positions but " + std::to_string(trajectory.joint_names.size()) + " joint names.");
      return BT::NodeStatus::FAILURE;
    }

    try
    {
      state.setVariablePositions(trajectory.joint_names, point.positions);
    }
    catch (const std::exception& ex)
    {
      shared_resources_->logger->publishFailureMessage(
          name(), std::string("Failed to set joint positions on robot state: ") + ex.what());
      return BT::NodeStatus::FAILURE;
    }
    state.updateLinkTransforms();

    const Eigen::Isometry3d link_pose = state.getGlobalLinkTransform(tip_link);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame_id;
    pose_msg.header.stamp = stamp;
    pose_msg.pose = tf2::toMsg(link_pose);
    path.push_back(pose_msg);
  }

  if (path.size() < 2)
  {
    shared_resources_->logger->publishWarnMessage(
        name(), "Trajectory has fewer than 2 points (" + std::to_string(path.size()) +
                    "); the resulting path will not render as a polyline.");
  }

  setOutput(kPortIDPath, path);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
