#include "geometry_behaviors/orient_pose_axis_to_reference_axis.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

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
constexpr auto kPortIDAxisToOrient = "axis_to_orient";
constexpr auto kPortIDTargetAxis = "target_axis";
constexpr auto kPortIDAllowAxisFlip = "allow_axis_flip";
constexpr auto kPortIDReorientedPose = "reoriented_pose";

const std::unordered_set<std::string> kValidAxes{ "+X", "-X", "X", "+Y", "-Y", "Y", "+Z", "-Z", "Z" };

// Map an axis name to its body-frame unit vector. Accepts "+X"/"-X"/"X" (= "+X")
// and the same for Y/Z. Returns nullopt for any other input.
std::optional<Eigen::Vector3d> axisFromString(const std::string& s)
{
  if (s == "+X" || s == "X")
    return Eigen::Vector3d::UnitX();
  if (s == "-X")
    return -Eigen::Vector3d::UnitX();
  if (s == "+Y" || s == "Y")
    return Eigen::Vector3d::UnitY();
  if (s == "-Y")
    return -Eigen::Vector3d::UnitY();
  if (s == "+Z" || s == "Z")
    return Eigen::Vector3d::UnitZ();
  if (s == "-Z")
    return -Eigen::Vector3d::UnitZ();
  return std::nullopt;
}

// Shortest rotation that takes a onto b. Handles parallel/anti-parallel degeneracies.
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

// Swing-only alignment: rotate QA by the minimal-angle rotation that takes QA's chosen body axis
// onto QB's chosen body axis. With allow_axis_flip, aligns to whichever of ±target_axis is closer.
Eigen::Quaterniond alignAxisSwing(const Eigen::Quaterniond& QA, const Eigen::Quaterniond& QB,
                                  const Eigen::Vector3d& axis_A_body, const Eigen::Vector3d& axis_B_body,
                                  bool allow_axis_flip)
{
  Eigen::Quaterniond qA = QA.normalized();
  Eigen::Quaterniond qB = QB.normalized();

  Eigen::Vector3d aA = qA * axis_A_body;
  Eigen::Vector3d aB = qB * axis_B_body;

  if (allow_axis_flip && aA.dot(aB) < 0.0)
    aB = -aB;

  const Eigen::Quaterniond rSwing = shortestRot(aA, aB);
  return (rSwing * qA).normalized();
}

Eigen::Quaterniond eigenQuatFromRos(const geometry_msgs::msg::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

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

OrientPoseAxisToReferenceAxis::OrientPoseAxisToReferenceAxis(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList OrientPoseAxisToReferenceAxis::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseToOrient, "{pose_to_orient}", "The pose to orient."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDReferencePose, "{reference_pose}",
                                                       "The reference pose to orient to."),
        BT::InputPort<std::string>(kPortIDAxisToOrient, "{axis_to_orient}",
                                   "Body-frame axis of pose_to_orient to align (+X/-X/+Y/-Y/+Z/-Z; "
                                   "X/Y/Z are aliases for +X/+Y/+Z)."),
        BT::InputPort<std::string>(kPortIDTargetAxis, "{target_axis}",
                                   "Body-frame axis of reference_pose to align onto (same alphabet as "
                                   "axis_to_orient)."),
        BT::InputPort<bool>(kPortIDAllowAxisFlip, true,
                            "If true (default), align to ±target_axis whichever requires the smaller rotation. "
                            "If false, align strictly to the signed target_axis."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDReorientedPose, "{reoriented_pose}",
                                                        "The reoriented pose") });
}

BT::KeyValueVector OrientPoseAxisToReferenceAxis::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Reorients a pose by the minimal rotation that aligns one of its body axes with a chosen "
             "body axis of a reference pose. Position is unchanged. Optionally allows ± flip on the target "
             "axis so the output aligns to whichever sign requires less rotation." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" } };
}

BT::NodeStatus OrientPoseAxisToReferenceAxis::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseToOrient),
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDReferencePose), getInput<std::string>(kPortIDAxisToOrient),
      getInput<std::string>(kPortIDTargetAxis), getInput<bool>(kPortIDAllowAxisFlip));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "OrientPoseAxisToReferenceAxis: missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose_to_orient, reference_pose, axis_to_orient, target_axis, allow_axis_flip] = ports.value();

  if (!kValidAxes.count(axis_to_orient))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Invalid axis_to_orient '" + axis_to_orient + "'. Valid: +X, -X, +Y, -Y, +Z, -Z (X/Y/Z = +X/+Y/+Z).");
    return BT::NodeStatus::FAILURE;
  }
  if (!kValidAxes.count(target_axis))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Invalid target_axis '" + target_axis + "'. Valid: +X, -X, +Y, -Y, +Z, -Z (X/Y/Z = +X/+Y/+Z).");
    return BT::NodeStatus::FAILURE;
  }

  const std::string& pose_to_orient_frame = pose_to_orient.header.frame_id;
  const std::string& reference_pose_frame = reference_pose.header.frame_id;

  if (pose_to_orient_frame.empty() || reference_pose_frame.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "frame_id empty (source='" + pose_to_orient_frame +
                                                                    "', target='" + reference_pose_frame + "')");
    return BT::NodeStatus::FAILURE;
  }

  auto poseA_in_ref = pose_to_orient;
  if (pose_to_orient_frame != reference_pose_frame)
  {
    if (!getBehaviorContext()->transform_buffer_ptr->canTransform(reference_pose_frame, pose_to_orient_frame,
                                                                  tf2::TimePointZero))
    {
      getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot transform from '" + pose_to_orient_frame +
                                                                      "' to '" + reference_pose_frame + "'.");
      return BT::NodeStatus::FAILURE;
    }

    try
    {
      const auto T_ref_src = getBehaviorContext()->transform_buffer_ptr->lookupTransform(
          reference_pose_frame, pose_to_orient_frame, tf2::TimePointZero);
      tf2::doTransform(pose_to_orient, poseA_in_ref, T_ref_src);
    }
    catch (const std::exception& e)
    {
      getBehaviorContext()->logger->publishFailureMessage(name(), std::string("Transform to reference frame failed: ") +
                                                                      e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto aA_body_opt = axisFromString(axis_to_orient);
  const auto aB_body_opt = axisFromString(target_axis);
  if (!aA_body_opt || !aB_body_opt)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to map axes to vectors.");
    return BT::NodeStatus::FAILURE;
  }

  const Eigen::Quaterniond qA = eigenQuatFromRos(poseA_in_ref.pose.orientation);
  const Eigen::Quaterniond qB = eigenQuatFromRos(reference_pose.pose.orientation);

  const Eigen::Quaterniond qA_aligned =
      alignAxisSwing(qA, qB, aA_body_opt.value(), aB_body_opt.value(), allow_axis_flip);

  geometry_msgs::msg::PoseStamped reoriented_pose_ref = poseA_in_ref;
  reoriented_pose_ref.pose.orientation = rosQuatFromEigen(qA_aligned);

  auto reoriented_pose_out = reoriented_pose_ref;
  if (pose_to_orient_frame != reference_pose_frame)
  {
    try
    {
      const auto T_src_tgt = getBehaviorContext()->transform_buffer_ptr->lookupTransform(
          pose_to_orient_frame, reference_pose_frame, tf2::TimePointZero);
      tf2::doTransform(reoriented_pose_ref, reoriented_pose_out, T_src_tgt);
    }
    catch (const std::exception& e)
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), std::string("Transform back to source frame failed: ") + e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  setOutput(kPortIDReorientedPose, reoriented_pose_out);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
