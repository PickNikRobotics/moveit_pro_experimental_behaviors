// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/pose_to_vectors.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kDescriptionPoseToVectors = R"(
                <p>
                    Decomposes a PoseStamped into translation [x, y, z] and quaternion [x, y, z, w]
                    std::vector&lt;double&gt; outputs. Useful for feeding the vector-typed ports of behaviors
                    like TransformPose without resorting to a multi-step Unpack+InsertInVector subtree.
                </p>
                <p>
                    Inverse of VectorsToPose.
                </p>
            )";

constexpr auto kPortIDPoseStamped = "pose_stamped";
constexpr auto kPortIDTranslationXyz = "translation_xyz";
constexpr auto kPortIDQuaternionXyzw = "quaternion_xyzw";
}  // namespace

namespace experimental_behaviors
{
namespace detail
{
std::pair<std::vector<double>, std::vector<double>> decomposePose(const geometry_msgs::msg::Pose& pose)
{
  return { std::vector<double>{ pose.position.x, pose.position.y, pose.position.z },
           std::vector<double>{ pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w } };
}
}  // namespace detail

PoseToVectors::PoseToVectors(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList PoseToVectors::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped, "{pose_stamped}",
                                                          "PoseStamped to decompose."),
           BT::OutputPort<std::vector<double>>(kPortIDTranslationXyz, "{translation_xyz}",
                                               "[x, y, z] of the pose's position."),
           BT::OutputPort<std::vector<double>>(kPortIDQuaternionXyzw, "{quaternion_xyzw}",
                                               "[x, y, z, w] of the pose's orientation.") };
}

BT::KeyValueVector PoseToVectors::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Pose Handling" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionPoseToVectors } };
}

BT::NodeStatus PoseToVectors::tick()
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
  auto [translation, quaternion] = detail::decomposePose(pose_stamped.pose);

  setOutput(kPortIDTranslationXyz, translation);
  setOutput(kPortIDQuaternionXyzw, quaternion);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
