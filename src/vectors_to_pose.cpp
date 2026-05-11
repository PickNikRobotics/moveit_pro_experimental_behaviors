// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/vectors_to_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kDescriptionVectorsToPose = R"(
                <p>
                    Assembles a PoseStamped from translation [x, y, z] and quaternion [x, y, z, w]
                    std::vector&lt;double&gt; inputs plus a frame_id. Inverse of PoseToVectors.
                </p>
                <p>
                    Returns FAILURE if translation_xyz is not size 3 or quaternion_xyzw is not size 4.
                    The output PoseStamped's header.stamp is the node's current ROS time.
                </p>
            )";

constexpr auto kPortIDTranslationXyz = "translation_xyz";
constexpr auto kPortIDQuaternionXyzw = "quaternion_xyzw";
constexpr auto kPortIDFrameId = "frame_id";
constexpr auto kPortIDPoseStamped = "pose_stamped";
}  // namespace

namespace experimental_behaviors
{
namespace detail
{
tl::expected<geometry_msgs::msg::Pose, std::string> composePose(const std::vector<double>& translation_xyz,
                                                                const std::vector<double>& quaternion_xyzw)
{
  if (translation_xyz.size() != 3)
  {
    return tl::make_unexpected(fmt::format("translation_xyz must have 3 elements; got {}.", translation_xyz.size()));
  }
  if (quaternion_xyzw.size() != 4)
  {
    return tl::make_unexpected(fmt::format("quaternion_xyzw must have 4 elements; got {}.", quaternion_xyzw.size()));
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = translation_xyz[0];
  pose.position.y = translation_xyz[1];
  pose.position.z = translation_xyz[2];
  pose.orientation.x = quaternion_xyzw[0];
  pose.orientation.y = quaternion_xyzw[1];
  pose.orientation.z = quaternion_xyzw[2];
  pose.orientation.w = quaternion_xyzw[3];
  return pose;
}
}  // namespace detail

VectorsToPose::VectorsToPose(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList VectorsToPose::providedPorts()
{
  return { BT::InputPort<std::vector<double>>(kPortIDTranslationXyz, std::vector<double>{ 0.0, 0.0, 0.0 },
                                              "[x, y, z] of the pose's position."),
           BT::InputPort<std::vector<double>>(kPortIDQuaternionXyzw, std::vector<double>{ 0.0, 0.0, 0.0, 1.0 },
                                              "[x, y, z, w] of the pose's orientation."),
           BT::InputPort<std::string>(kPortIDFrameId, "world", "Frame the resulting PoseStamped is expressed in."),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped, "{pose_stamped}",
                                                           "Resulting PoseStamped.") };
}

BT::KeyValueVector VectorsToPose::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Pose Handling" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionVectorsToPose } };
}

BT::NodeStatus VectorsToPose::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::vector<double>>(kPortIDTranslationXyz),
                                                              getInput<std::vector<double>>(kPortIDQuaternionXyzw),
                                                              getInput<std::string>(kPortIDFrameId));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     "Failed to get required value from input port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [translation_xyz, quaternion_xyzw, frame_id] = ports.value();

  const auto pose_or_error = detail::composePose(translation_xyz, quaternion_xyzw);
  if (!pose_or_error.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), pose_or_error.error());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp = shared_resources_->node->now();
  pose_stamped.pose = pose_or_error.value();

  setOutput(kPortIDPoseStamped, pose_stamped);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
