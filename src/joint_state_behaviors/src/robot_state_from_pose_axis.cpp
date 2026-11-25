// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "joint_state_behaviors/robot_state_from_pose_axis.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <fmt/format.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>

namespace joint_state_behaviors
{

RobotStateFromPoseAxis::RobotStateFromPoseAxis(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList RobotStateFromPoseAxis::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose", "{input_pose}",
                                                          "Pose whose translation supplies the joint value."),
           BT::InputPort<std::string>("axis", "X", "Which translation component to take: X, Y or Z."),
           BT::InputPort<std::string>("joint_name", "The single joint the goal names."),
           BT::InputPort<double>("offset", "0.0", "Added to the coordinate before it becomes the joint value."),
           BT::OutputPort<moveit_msgs::msg::RobotState>(
               "robot_state", "{robot_state}", "One-joint goal, ready for Move to Joint State's target_joint_state.") };
}

BT::KeyValueVector RobotStateFromPoseAxis::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Builds a one-joint RobotState whose value is a translation component of a pose, for lining a prismatic "
             "axis up with something the tree only knows as a pose." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" } };
}

BT::NodeStatus RobotStateFromPoseAxis::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>("input_pose"),
                                               getInput<std::string>("axis"), getInput<std::string>("joint_name"),
                                               getInput<double>("offset"));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [pose, axis, joint_name, offset] = ports.value();

  double value = 0.0;
  if (axis == "X" || axis == "x")
  {
    value = pose.pose.position.x;
  }
  else if (axis == "Y" || axis == "y")
  {
    value = pose.pose.position.y;
  }
  else if (axis == "Z" || axis == "z")
  {
    value = pose.pose.position.z;
  }
  else
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "axis must be X, Y or Z, got '" + axis + "'.");
    return BT::NodeStatus::FAILURE;
  }
  value += offset;

  moveit_msgs::msg::RobotState out;
  out.joint_state.name.push_back(joint_name);
  out.joint_state.position.push_back(value);
  setOutput("robot_state", out);

  getBehaviorContext()->logger->publishInfoMessage(
      name(), fmt::format("{} := {:.4f} (from pose {} = {:.4f}{})", joint_name, value, axis, value - offset,
                          offset == 0.0 ? "" : fmt::format(", offset {:+.4f}", offset)));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace joint_state_behaviors
