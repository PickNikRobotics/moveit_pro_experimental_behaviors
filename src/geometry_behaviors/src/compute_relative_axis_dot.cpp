#include "geometry_behaviors/compute_relative_axis_dot.hpp"

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
constexpr auto kPortIDPoseA = "pose_a";
constexpr auto kPortIDPoseB = "pose_b";
constexpr auto kPortIDAxisA = "axis_a";
constexpr auto kPortIDAxisB = "axis_b";
constexpr auto kPortIDAbsDot = "abs_dot";

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

Eigen::Quaterniond eigenQuatFromRos(const geometry_msgs::msg::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}
}  // namespace

namespace geometry_behaviors
{

ComputeRelativeAxisDot::ComputeRelativeAxisDot(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList ComputeRelativeAxisDot::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseA, "{pose_a}", "First pose."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseB, "{pose_b}", "Second pose."),
        BT::InputPort<std::string>(kPortIDAxisA, "{axis_a}",
                                   "Body-frame axis of pose_a (+X/-X/+Y/-Y/+Z/-Z; X/Y/Z = +X/+Y/+Z)."),
        BT::InputPort<std::string>(kPortIDAxisB, "{axis_b}", "Body-frame axis of pose_b (same alphabet as axis_a)."),
        BT::OutputPort<double>(kPortIDAbsDot, "{abs_dot}",
                               "Absolute value of the dot product of the two chosen axes, expressed in a common "
                               "frame. 1.0 = parallel (any sign), 0.0 = perpendicular.") });
}

BT::KeyValueVector ComputeRelativeAxisDot::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Computes |dot(axis_a, axis_b)| where each axis is the named body-frame axis of its pose, "
             "expressed in a common frame. Use to discriminate parallel-vs-perpendicular alignment between "
             "two poses without caring about sign." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" } };
}

BT::NodeStatus ComputeRelativeAxisDot::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseA),
                                                              getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseB),
                                                              getInput<std::string>(kPortIDAxisA),
                                                              getInput<std::string>(kPortIDAxisB));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "ComputeRelativeAxisDot: missing required input: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose_a, pose_b, axis_a, axis_b] = ports.value();

  if (!kValidAxes.count(axis_a))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Invalid axis_a '" + axis_a + "'. Valid: +X, -X, +Y, -Y, +Z, -Z (X/Y/Z = +X/+Y/+Z).");
    return BT::NodeStatus::FAILURE;
  }
  if (!kValidAxes.count(axis_b))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Invalid axis_b '" + axis_b + "'. Valid: +X, -X, +Y, -Y, +Z, -Z (X/Y/Z = +X/+Y/+Z).");
    return BT::NodeStatus::FAILURE;
  }

  const std::string& pose_a_frame = pose_a.header.frame_id;
  const std::string& pose_b_frame = pose_b.header.frame_id;

  if (pose_a_frame.empty() || pose_b_frame.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "frame_id empty (pose_a='" + pose_a_frame +
                                                                    "', pose_b='" + pose_b_frame + "')");
    return BT::NodeStatus::FAILURE;
  }

  // Express both quaternions in pose_b's frame so body→common-frame rotations agree.
  auto poseA_in_b = pose_a;
  if (pose_a_frame != pose_b_frame)
  {
    if (!getBehaviorContext()->transform_buffer_ptr->canTransform(pose_b_frame, pose_a_frame, tf2::TimePointZero))
    {
      getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot transform from '" + pose_a_frame + "' to '" +
                                                                      pose_b_frame + "'.");
      return BT::NodeStatus::FAILURE;
    }
    try
    {
      const auto T_b_a =
          getBehaviorContext()->transform_buffer_ptr->lookupTransform(pose_b_frame, pose_a_frame, tf2::TimePointZero);
      tf2::doTransform(pose_a, poseA_in_b, T_b_a);
    }
    catch (const std::exception& e)
    {
      getBehaviorContext()->logger->publishFailureMessage(name(),
                                                          std::string("Transform to common frame failed: ") + e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto aA_body_opt = axisFromString(axis_a);
  const auto aB_body_opt = axisFromString(axis_b);
  if (!aA_body_opt || !aB_body_opt)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to map axes to vectors.");
    return BT::NodeStatus::FAILURE;
  }

  const Eigen::Quaterniond qA = eigenQuatFromRos(poseA_in_b.pose.orientation);
  const Eigen::Quaterniond qB = eigenQuatFromRos(pose_b.pose.orientation);

  const Eigen::Vector3d aA_world = (qA * aA_body_opt.value()).normalized();
  const Eigen::Vector3d aB_world = (qB * aB_body_opt.value()).normalized();

  const double abs_dot = std::abs(std::clamp(aA_world.dot(aB_world), -1.0, 1.0));

  setOutput(kPortIDAbsDot, abs_dot);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
