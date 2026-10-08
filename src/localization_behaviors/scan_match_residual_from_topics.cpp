// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/localization_behaviors/scan_match_residual_from_topics.hpp>

#include <fmt/format.h>
#include <tf2/exceptions.h>
#include <experimental_behaviors/localization_behaviors/scan_match.hpp>
#include <experimental_behaviors/localization_behaviors/wait_for_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>

namespace
{
constexpr auto kPortPose = "pose";
constexpr auto kPortRobotFrame = "robot_frame_id";
constexpr auto kPortScanTopic = "scan_topic";
constexpr auto kPortMapTopic = "map_topic";
constexpr auto kPortInlierDistance = "inlier_distance";
constexpr auto kPortMinInlierFraction = "min_inlier_fraction";
constexpr auto kPortTimeout = "message_timeout_sec";
constexpr auto kPortInlierFraction = "inlier_fraction";
constexpr auto kPortMedianResidual = "median_residual";
constexpr auto kPortBeamsUsed = "beams_used";

constexpr auto kDefaultTimeoutSeconds = 5.0;
// Occupied on the OccupancyGrid 0 to 100 scale, matching map_server's usual occupied_thresh.
constexpr int kOccupiedThreshold = 65;

constexpr auto kDescription = R"(
    <p>Scores a robot pose against the latest <code>sensor_msgs/LaserScan</code> and a
    <code>nav_msgs/OccupancyGrid</code> map, read from their topics, and fails when the fit is poor.</p>
    <p>Each usable beam is placed in the map through the pose and the TF offset from
    <code>robot_frame_id</code> to the scan frame. Its residual is the distance from the beam endpoint to the
    nearest occupied cell, and it is an inlier when that is within <code>inlier_distance</code>. Max-range and
    invalid returns are skipped. An endpoint off the map is an outlier.</p>
    <p>Outputs the inlier fraction, the median residual and the number of beams used, then SUCCESS when the
    inlier fraction is at least <code>min_inlier_fraction</code>. FAILURE otherwise, or if the pose is not finite or
    not in the map frame, an input is out of range, the TF offset is unavailable, no beam is usable, or a message
    does not arrive in time.</p>
)";

bool isFinite(const geometry_msgs::msg::Pose& pose)
{
  const auto& p = pose.position;
  const auto& q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}
}  // namespace

namespace experimental_behaviors
{
ScanMatchResidualFromTopics::ScanMatchResidualFromTopics(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources), context_{ shared_resources }
{
}

BT::PortsList ScanMatchResidualFromTopics::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "Robot pose to score, in the map's frame."),
    BT::InputPort<std::string>(kPortRobotFrame, "base_link",
                               "TF frame the pose places, whose offset to the scan frame is looked up."),
    BT::InputPort<std::string>(kPortScanTopic, "/scan", "sensor_msgs/LaserScan topic."),
    BT::InputPort<std::string>(kPortMapTopic, "/map", "nav_msgs/OccupancyGrid topic."),
    BT::InputPort<double>(kPortInlierDistance, "Largest endpoint-to-obstacle distance that is an inlier (m)."),
    BT::InputPort<double>(kPortMinInlierFraction, "Smallest inlier fraction, from 0 to 1, that passes."),
    BT::InputPort<double>(kPortTimeout, kDefaultTimeoutSeconds, "Seconds to wait for each of the scan and the map."),
    BT::OutputPort<double>(kPortInlierFraction, "{inlier_fraction}", "Fraction of used beams that are inliers."),
    BT::OutputPort<double>(kPortMedianResidual, "{median_residual}", "Median endpoint residual (m)."),
    BT::OutputPort<int>(kPortBeamsUsed, "{beams_used}", "Number of beams scored."),
  };
}

BT::KeyValueVector ScanMatchResidualFromTopics::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Navigation" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

tl::expected<bool, std::string> ScanMatchResidualFromTopics::doWork()
{
  cancel_ = false;
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortPose), getInput<std::string>(kPortRobotFrame),
      getInput<std::string>(kPortScanTopic), getInput<std::string>(kPortMapTopic),
      getInput<double>(kPortInlierDistance), getInput<double>(kPortMinInlierFraction), getInput<double>(kPortTimeout));
  if (!ports)
  {
    return tl::make_unexpected("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [pose, robot_frame, scan_topic, map_topic, inlier_distance, min_inlier_fraction, timeout] = ports.value();
  if (!std::isfinite(inlier_distance) || inlier_distance <= 0.0 ||
      !(min_inlier_fraction >= 0.0 && min_inlier_fraction <= 1.0))
  {
    return tl::make_unexpected(fmt::format("inlier_distance must be finite and positive and min_inlier_fraction in "
                                           "[0, 1], got {} and {}.",
                                           inlier_distance, min_inlier_fraction));
  }
  if (!isFinite(pose.pose))
  {
    return tl::make_unexpected("Pose has a non-finite position or orientation.");
  }

  notifyCanHalt();
  const std::chrono::duration<double> wait{ timeout };
  const auto map = localization_utils::waitForMessage<nav_msgs::msg::OccupancyGrid>(
      context_->node, context_->reentrant_callback_group, map_topic, wait, cancel_);
  if (!map)
  {
    return tl::make_unexpected(map.error());
  }
  if (pose.header.frame_id != map->header.frame_id)
  {
    return tl::make_unexpected("Pose is in frame '" + pose.header.frame_id + "' but the map is in '" +
                               map->header.frame_id + "'. Transform the pose into the map frame first.");
  }
  const auto scan = localization_utils::waitForMessage<sensor_msgs::msg::LaserScan>(
      context_->node, context_->reentrant_callback_group, scan_topic, wait, cancel_);
  if (!scan)
  {
    return tl::make_unexpected(scan.error());
  }

  // The robot is assumed still, so the latest offset is used rather than one at the scan's stamp.
  Eigen::Isometry3d scan_in_robot;
  try
  {
    scan_in_robot = tf2::transformToEigen(
        context_->transform_buffer_ptr->lookupTransform(robot_frame, scan->header.frame_id, tf2::TimePointZero));
  }
  catch (const tf2::TransformException& e)
  {
    return tl::make_unexpected("Failed to look up the scan frame '" + scan->header.frame_id + "' in '" + robot_frame +
                               "': " + e.what());
  }
  Eigen::Isometry3d robot_in_map;
  tf2::fromMsg(pose.pose, robot_in_map);

  const localization_utils::OccupancyDistanceField field{ map.value(), kOccupiedThreshold };
  const auto score = localization_utils::scoreScan(scan.value(), robot_in_map * scan_in_robot, field, inlier_distance);
  setOutput(kPortInlierFraction, score.inlier_fraction);
  setOutput(kPortMedianResidual, score.median_residual);
  setOutput(kPortBeamsUsed, static_cast<int>(score.beams_used));

  if (score.beams_used == 0)
  {
    return tl::make_unexpected("No usable beam in the scan on '" + scan_topic + "'.");
  }
  if (score.inlier_fraction < min_inlier_fraction)
  {
    return tl::make_unexpected(fmt::format("Scan does not match the map at this pose: inlier fraction {:.3f} is "
                                           "below {:.3f} ({} beams, median residual {:.3f} m).",
                                           score.inlier_fraction, min_inlier_fraction, score.beams_used,
                                           score.median_residual));
  }
  return true;
}

tl::expected<void, std::string> ScanMatchResidualFromTopics::doHalt()
{
  cancel_ = true;
  return {};
}
}  // namespace experimental_behaviors
