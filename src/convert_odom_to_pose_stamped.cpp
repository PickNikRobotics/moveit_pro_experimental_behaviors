// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/convert_odom_to_pose_stamped.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIDOdometry = "odometry";
constexpr auto kPortIDPoseStamped = "pose_stamped";
}  // namespace

namespace experimental_behaviors
{
ConvertOdomToPoseStamped::ConvertOdomToPoseStamped(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ConvertOdomToPoseStamped::providedPorts()
{
  return { BT::InputPort<nav_msgs::msg::Odometry>(kPortIDOdometry, "{odometry}",
                                                   "The nav_msgs::msg::Odometry message to convert."),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>(
               kPortIDPoseStamped, "{pose_stamped}",
               "The converted geometry_msgs::msg::PoseStamped message.") };
}

BT::KeyValueVector ConvertOdomToPoseStamped::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Conversions" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Converts a nav_msgs::msg::Odometry message into a geometry_msgs::msg::PoseStamped "
             "message. Header and pose are copied; the twist field is dropped." } };
}

BT::NodeStatus ConvertOdomToPoseStamped::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<nav_msgs::msg::Odometry>(kPortIDOdometry));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     "Failed to get required value from input port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [odom_msg] = ports.value();

  geometry_msgs::msg::PoseStamped pose_out;
  pose_out.header = odom_msg.header;
  pose_out.pose = odom_msg.pose.pose;

  setOutput(kPortIDPoseStamped, pose_out);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
