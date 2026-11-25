// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "grasping_behaviors/generate_place_candidates.hpp"

#include <cmath>
#include <utility>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
constexpr auto kPortIDObjectPose = "object_pose";
constexpr auto kPortIDPickPose = "pick_pose";
constexpr auto kPortIDTargetPose = "target_pose";
constexpr auto kPortIDTargetOffset = "target_offset_xyz";
constexpr auto kPortIDYawRange = "yaw_range_deg";
constexpr auto kPortIDYawIncrement = "yaw_increment_deg";
constexpr auto kPortIDIncludeUpright = "include_upright";
constexpr auto kPortIDIncludeFlipped = "include_flipped";
constexpr auto kPortIDPlacePoses = "place_poses";
constexpr auto kPortIDPredictedObjectPose = "predicted_object_pose";

constexpr double kDefaultYawIncrementDeg = 20.0;
// Tolerance on range / increment, so that an exact multiple does not lose its last step to rounding.
constexpr double kStepCountEpsilon = 1e-9;

Eigen::Isometry3d toIsometry(const geometry_msgs::msg::PoseStamped& pose)
{
  Eigen::Isometry3d isometry;
  tf2::fromMsg(pose.pose, isometry);
  return isometry;
}
}  // namespace

namespace grasping_behaviors
{
namespace internal
{
tl::expected<PlaceCandidates, std::string> generatePlaceCandidates(const PlaceCandidateRequest& request)
{
  if (!request.include_upright && !request.include_flipped)
  {
    return tl::make_unexpected("include_upright and include_flipped are both false, so there is no candidate");
  }

  // Yaw offsets in nominal-first order: 0, then -step, +step, -2 step, +2 step and so on.
  std::vector<double> yaw_offsets_rad = { 0.0 };
  const double range_deg = std::max(0.0, request.yaw_range_deg);
  if (range_deg > 0.0)
  {
    if (!(request.yaw_increment_deg > 0.0))
    {
      return tl::make_unexpected("yaw_increment_deg must be greater than zero when yaw_range_deg is set");
    }
    const int steps_per_side = static_cast<int>(std::floor(range_deg / request.yaw_increment_deg + kStepCountEpsilon));
    for (int i = 1; i <= steps_per_side; ++i)
    {
      const double offset_rad = i * request.yaw_increment_deg * M_PI / 180.0;
      yaw_offsets_rad.push_back(-offset_rad);
      yaw_offsets_rad.push_back(offset_rad);
    }
  }

  const Eigen::Isometry3d grasp_to_object = request.grasp_at_pick.inverse() * request.object_at_pick;
  const Eigen::Isometry3d object_desired = request.target * Eigen::Translation3d(request.target_offset);

  std::vector<Eigen::Isometry3d> object_poses;
  if (request.include_upright)
  {
    object_poses.push_back(object_desired);
  }
  if (request.include_flipped)
  {
    object_poses.push_back(object_desired * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  PlaceCandidates result;
  result.predicted_object_pose = object_poses.front();
  for (const auto& object_pose : object_poses)
  {
    for (const double yaw : yaw_offsets_rad)
    {
      // Turn about the target Z axis through the place point; right-multiplying would turn about the object's own Z.
      const Eigen::Isometry3d turned = Eigen::Translation3d(object_desired.translation()) *
                                       Eigen::AngleAxisd(yaw, object_desired.linear().col(2)) *
                                       Eigen::Translation3d(-object_desired.translation()) * object_pose;
      result.grasp_poses.push_back(turned * grasp_to_object.inverse());
    }
  }
  return result;
}
}  // namespace internal

GeneratePlaceCandidates::GeneratePlaceCandidates(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GeneratePlaceCandidates::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDObjectPose, "{object_pose}",
                                                   "Pose of the object when it was picked, for example from detection. "
                                                   "Must be in the same frame as pick_pose."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPickPose, "{pick_pose}",
                                                   "Pose of the grasp link when the object was picked."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDTargetPose, "{target_pose}",
                                                   "Pose that the object frame must have after the place."),
    BT::InputPort<std::vector<double>>(kPortIDTargetOffset, "0;0;0",
                                       "Offset \"x;y;z\" of the object frame from target_pose, in the target frame, "
                                       "for example half the object height along Z."),
    BT::InputPort<double>(kPortIDYawRange, 0.0,
                          "Half width, in degrees, of a window of turns about the target Z axis. Zero gives only the "
                          "nominal orientation."),
    BT::InputPort<double>(kPortIDYawIncrement, kDefaultYawIncrementDeg,
                          "Step, in degrees, between the turns inside the yaw window."),
    BT::InputPort<bool>(kPortIDIncludeUpright, true,
                        "Output candidates that keep the object as target_pose orients it."),
    BT::InputPort<bool>(kPortIDIncludeFlipped, false,
                        "Also output candidates with the object upside down (a half turn about the target X axis). "
                        "Use it only when the object may be placed upside down."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPlacePoses, "{place_poses}",
                                                                 "Grasp-link poses that place the object, in the frame "
                                                                 "of target_pose. Upright candidates first, nominal "
                                                                 "orientation first."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDPredictedObjectPose, "{predicted_object_pose}",
                                                    "Object pose for the first candidate, for visualization."),
  };
}

BT::KeyValueVector GeneratePlaceCandidates::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Grasping" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Computes grasp-link poses that place a held object at a target pose, from the object and grasp-link "
             "poses at pick time. Optional extra candidates turn the object about the target Z axis or place it "
             "upside down." } };
}

BT::NodeStatus GeneratePlaceCandidates::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDObjectPose),
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDPickPose),
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDTargetPose), getInput<std::vector<double>>(kPortIDTargetOffset),
      getInput<double>(kPortIDYawRange), getInput<double>(kPortIDYawIncrement), getInput<bool>(kPortIDIncludeUpright),
      getInput<bool>(kPortIDIncludeFlipped));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [object_pose, pick_pose, target_pose, target_offset, yaw_range, yaw_increment, include_upright,
               include_flipped] = ports.value();

  if (object_pose.header.frame_id != pick_pose.header.frame_id)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "object_pose (frame '" + object_pose.header.frame_id + "') and pick_pose (frame '" +
                    pick_pose.header.frame_id + "') must be in the same frame.");
    return BT::NodeStatus::FAILURE;
  }
  if (target_offset.size() != 3)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(),
                                                        "target_offset_xyz needs exactly three values, \"x;y;z\".");
    return BT::NodeStatus::FAILURE;
  }

  internal::PlaceCandidateRequest request;
  request.object_at_pick = toIsometry(object_pose);
  request.grasp_at_pick = toIsometry(pick_pose);
  request.target = toIsometry(target_pose);
  request.target_offset = Eigen::Vector3d(target_offset[0], target_offset[1], target_offset[2]);
  request.yaw_range_deg = yaw_range;
  request.yaw_increment_deg = yaw_increment;
  request.include_upright = include_upright;
  request.include_flipped = include_flipped;

  const auto result = internal::generatePlaceCandidates(request);
  if (!result.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), result.error());
    return BT::NodeStatus::FAILURE;
  }

  // The math is in the frame of target_pose, so every output carries that frame.
  std::vector<geometry_msgs::msg::PoseStamped> place_poses;
  place_poses.reserve(result->grasp_poses.size());
  for (const auto& grasp_pose : result->grasp_poses)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = target_pose.header.frame_id;
    pose.pose = tf2::toMsg(grasp_pose);
    place_poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped predicted;
  predicted.header.frame_id = target_pose.header.frame_id;
  predicted.pose = tf2::toMsg(result->predicted_object_pose);

  setOutput(kPortIDPlacePoses, place_poses);
  setOutput(kPortIDPredictedObjectPose, predicted);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace grasping_behaviors
