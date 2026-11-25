// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "geometry_behaviors/orient_pose_plane_to_reference_plane.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_set>

namespace
{
constexpr auto kPortIDPoseToOrient = "pose_to_orient";
constexpr auto kPortIDReferencePose = "reference_pose";
constexpr auto kPortIDPlaneToOrient = "plane_to_orient";
constexpr auto kPortIDTargetPlane = "target_plane";
constexpr auto kPortIDReorientedPose = "reoriented_pose";

const std::unordered_set<std::string> kValidPlanes{ "XY", "XZ", "YZ" };

/**
 * @brief Map a plane name ("XY", "XZ", "YZ") to its body-frame unit normal.
 *
 * The body-frame convention is:
 *  - XY plane → +Z
 *  - XZ plane → +Y
 *  - YZ plane → +X
 *
 * @param p Plane string ("XY", "XZ", "YZ").
 * @return Body-frame normal vector if valid; std::nullopt otherwise.
 */
std::optional<Eigen::Vector3d> planeNormalFromString(const std::string& p)
{
  if (p == "XY")
    return Eigen::Vector3d::UnitZ();
  if (p == "XZ")
    return Eigen::Vector3d::UnitY();
  if (p == "YZ")
    return Eigen::Vector3d::UnitX();
  return std::nullopt;
}

/**
 * @brief Compute the shortest rotation that takes vector @p a onto vector @p b.
 *
 * Both @p a and @p b are interpreted in the same frame.
 * Handles the parallel and anti-parallel degeneracies robustly.
 *
 * @param a Source direction (need not be unit length).
 * @param b Target direction (need not be unit length).
 * @return Quaternion representing the minimal-angle rotation such that R * a = b (up to scale).
 *
 * @note If @p a and @p b are already parallel (dot>0), returns identity.
 *       If they are anti-parallel (dot<0 and cross≈0), returns a 180° rotation
 *       about an arbitrary axis orthogonal to @p a (stable choice implemented).
 */
Eigen::Quaterniond shortestRot(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d na = a.normalized();
  const Eigen::Vector3d nb = b.normalized();
  const double c = std::clamp(na.dot(nb), -1.0, 1.0);
  const Eigen::Vector3d v = na.cross(nb);
  const double s = v.norm();
  constexpr double eps = 1e-12;

  if (s < eps)
  {
    if (c > 0.0)
      return Eigen::Quaterniond::Identity();

    const Eigen::Vector3d arbitrary = (std::abs(na.y()) < 0.9) ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d axis = na.cross(arbitrary).normalized();
    return Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, axis));
  }

  const Eigen::Vector3d axis = v / s;
  const double angle = std::acos(c);
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

/**
 * @brief Make the selected plane of @p QA parallel to the selected plane of @p QB
 *        via a minimal-angle "swing" rotation (no twist control).
 *
 * @param QA Orientation of pose A
 * @param QB Orientation of pose B
 * @param plane_normal_A_body Body-frame normal of A’s plane (e.g., UnitZ for "XY").
 * @param plane_normal_B_body Body-frame normal of B’s plane.
 * @param choose_shortest_flip If true, allows flipping the target normal (nB→−nB)
 *        to minimize rotation magnitude (results in parallel or anti-parallel planes).
 * @return New orientation for A after applying the swing: qA' = r_swing * QA.
 *
 * @note Only the plane normals are aligned; rotation about the shared normal is left unconstrained.
 * @warning Assumes Eigen’s vector transform convention v_world = q * v_body.
 */
Eigen::Quaterniond alignPlaneSwing(const Eigen::Quaterniond& QA, const Eigen::Quaterniond& QB,
                                   const Eigen::Vector3d& plane_normal_A_body,
                                   const Eigen::Vector3d& plane_normal_B_body, bool choose_shortest_flip = true)
{
  Eigen::Quaterniond qA = QA.normalized();
  Eigen::Quaterniond qB = QB.normalized();

  Eigen::Vector3d nA = qA * plane_normal_A_body;
  Eigen::Vector3d nB = qB * plane_normal_B_body;

  if (choose_shortest_flip && nA.dot(nB) < 0.0)
    nB = -nB;

  const Eigen::Quaterniond rSwing = shortestRot(nA, nB);
  return (rSwing * qA).normalized();
}

/**
 * @brief Convert a ROS geometry_msgs Quaternion (x,y,z,w) to Eigen::Quaterniond (w,x,y,z).
 *
 * Normalizes the result to guard against non-unit inputs.
 *
 * @param q ROS quaternion.
 * @return Normalized Eigen quaternion.
 */
Eigen::Quaterniond eigenQuatFromRos(const geometry_msgs::msg::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

/**
 * @brief Convert an Eigen::Quaterniond (w,x,y,z) to a ROS geometry_msgs Quaternion (x,y,z,w).
 *
 * Normalizes the input before exporting to ensure unit output.
 *
 * @param q Eigen quaternion (will be normalized).
 * @return ROS quaternion.
 */
geometry_msgs::msg::Quaternion rosQuatFromEigen(const Eigen::Quaterniond& q)
{
  geometry_msgs::msg::Quaternion out;
  const Eigen::Quaterniond qq = q.normalized();
  out.x = qq.x();
  out.y = qq.y();
  out.z = qq.z();
  out.w = qq.w();
  return out;
}
}  // namespace

namespace geometry_behaviors
{

OrientPosePlaneToReferencePlane::OrientPosePlaneToReferencePlane(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList OrientPosePlaneToReferencePlane::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseToOrient, "{pose_to_orient}", "The pose to orient."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDReferencePose, "{reference_pose}",
                                                       "The reference pose to orient to."),
        BT::InputPort<std::string>(kPortIDPlaneToOrient, "{plane_to_orient}", "The plane we want to orient."),
        BT::InputPort<std::string>(kPortIDTargetPlane, "{target_plane}", "The plane we want to orient to."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDReorientedPose, "{reoriented_pose}",
                                                        "The reoriented pose") });
}

BT::KeyValueVector OrientPosePlaneToReferencePlane::metadata()
{
  return { { "description",
             "Reorients a pose through a minimal rotation so that a given plane is parallel to a reference plane." },
           { "subcategory", "Utility" } };
}

BT::NodeStatus OrientPosePlaneToReferencePlane::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseToOrient),
                                               getInput<geometry_msgs::msg::PoseStamped>(kPortIDReferencePose),
                                               getInput<std::string>(kPortIDPlaneToOrient),
                                               getInput<std::string>(kPortIDTargetPlane));

  if (!ports.has_value())
  {
    spdlog::warn("OrientPosePlaneToReferencePlane: missing required input: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose_to_orient, reference_pose, plane_to_orient, target_plane] = ports.value();

  // Validate planes
  if (!kValidPlanes.count(plane_to_orient))
  {
    spdlog::warn("OrientPosePlaneToReferencePlane: invalid plane_to_orient '{}'. Valid: XY, XZ, YZ", plane_to_orient);
    return BT::NodeStatus::FAILURE;
  }
  if (!kValidPlanes.count(target_plane))
  {
    spdlog::warn("OrientPosePlaneToReferencePlane: invalid target_plane '{}'. Valid: XY, XZ, YZ", target_plane);
    return BT::NodeStatus::FAILURE;
  }

  const std::string& pose_to_orient_frame = pose_to_orient.header.frame_id;
  const std::string& reference_pose_frame = reference_pose.header.frame_id;

  if (pose_to_orient_frame.empty() || reference_pose_frame.empty())
  {
    spdlog::warn("OrientPosePlaneToReferencePlane: frame_id empty (source='{}', target='{}')", pose_to_orient_frame,
                 reference_pose_frame);
    return BT::NodeStatus::FAILURE;
  }

  // Transform pose_to_orient into the reference frame if needed
  auto poseA_in_ref = pose_to_orient;
  if (pose_to_orient_frame != reference_pose_frame)
  {
    if (!getBehaviorContext()->transform_buffer_ptr->canTransform(reference_pose_frame, pose_to_orient_frame,
                                                                  tf2::TimePointZero))
    {
      spdlog::warn("OrientPosePlaneToReferencePlane: cannot transform from '{}' to '{}'.", pose_to_orient_frame,
                   reference_pose_frame);
      return BT::NodeStatus::FAILURE;
    }

    try
    {
      const auto T_ref_src = getBehaviorContext()->transform_buffer_ptr->lookupTransform(
          reference_pose_frame, pose_to_orient_frame, tf2::TimePointZero);
      tf2::doTransform(pose_to_orient, poseA_in_ref, T_ref_src);
      // poseA_in_ref now in reference_pose_frame
    }
    catch (const std::exception& e)
    {
      spdlog::warn("OrientPosePlaneToReferencePlane: transform to reference frame failed: {}", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto nA_body_opt = planeNormalFromString(plane_to_orient);
  const auto nB_body_opt = planeNormalFromString(target_plane);
  if (!nA_body_opt || !nB_body_opt)
  {
    spdlog::warn("OrientPosePlaneToReferencePlane: failed to map planes to normals.");
    return BT::NodeStatus::FAILURE;
  }

  // Convert quaternions (both expressed in reference_pose_frame now)
  const Eigen::Quaterniond qA = eigenQuatFromRos(poseA_in_ref.pose.orientation);
  const Eigen::Quaterniond qB = eigenQuatFromRos(reference_pose.pose.orientation);

  // Compute aligned orientation (swing-only; allow flip for minimal rotation)
  const Eigen::Quaterniond qA_aligned =
      alignPlaneSwing(qA, qB, nA_body_opt.value(), nB_body_opt.value(), /*choose_shortest_flip=*/true);

  // Compose the reoriented pose in the reference frame first
  geometry_msgs::msg::PoseStamped reoriented_pose_ref = poseA_in_ref;
  reoriented_pose_ref.pose.orientation = rosQuatFromEigen(qA_aligned);

  // Transform back to the original pose_to_orient_frame if needed
  auto reoriented_pose_out = reoriented_pose_ref;
  if (pose_to_orient_frame != reference_pose_frame)
  {
    try
    {
      // Convert pose in tgt -> src
      const auto T_src_tgt = getBehaviorContext()->transform_buffer_ptr->lookupTransform(
          pose_to_orient_frame, reference_pose_frame, tf2::TimePointZero);
      tf2::doTransform(reoriented_pose_ref, reoriented_pose_out, T_src_tgt);
      // reoriented_pose_out now in pose_to_orient_frame
    }
    catch (const std::exception& e)
    {
      spdlog::warn("OrientPosePlaneToReferencePlane: transform back to source frame failed: {}", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Emit final pose in the original frame (position unchanged, orientation reoriented)
  setOutput(kPortIDReorientedPose, reoriented_pose_out);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
