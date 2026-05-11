// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/convert_pose_stamped_to_odom.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIDPoseStamped = "pose_stamped";
constexpr auto kPortIDOdometry = "odom";
constexpr auto kPortIDChildFrame = "child_frame";
}  // namespace

namespace experimental_behaviors
{
ConvertPoseStampedToOdom::ConvertPoseStampedToOdom(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ConvertPoseStampedToOdom::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped, "{pose_stamped}",
                                                          "The PoseStamped message to convert."),
           BT::InputPort<std::string>(kPortIDChildFrame, "",
                                      "Optional child_frame_id for the Odometry message."),
           BT::OutputPort<nav_msgs::msg::Odometry>(kPortIDOdometry, "{odom}", "The converted Odometry message.") };
}

BT::KeyValueVector ConvertPoseStampedToOdom::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Conversions" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Converts a geometry_msgs::msg::PoseStamped message into a nav_msgs::msg::Odometry "
             "message. Header and pose are copied; twist is left zero-initialized." } };
}

BT::NodeStatus ConvertPoseStampedToOdom::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     "Failed to get required value from input port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [pose_stamped] = ports.value();

  nav_msgs::msg::Odometry odom_out;
  odom_out.header = pose_stamped.header;
  odom_out.pose.pose = pose_stamped.pose;

  // Optional child_frame_id — empty if the port wasn't set.
  if (const auto child_frame = getInput<std::string>(kPortIDChildFrame); child_frame.has_value())
  {
    odom_out.child_frame_id = child_frame.value();
  }

  // Twist is intentionally left zero-initialized.

  setOutput(kPortIDOdometry, odom_out);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
