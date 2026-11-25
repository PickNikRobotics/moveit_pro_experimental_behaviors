#include "grasping_behaviors/determine_pick_alignment.hpp"

#include <cmath>
#include <string>

#include <spdlog/spdlog.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
constexpr auto kPortIDSolidPrimitive = "solid_primitive";
constexpr auto kPortIDPrimitivePose = "primitive_pose";
constexpr auto kPortIDGraspPose = "grasp_pose";
constexpr auto kPortIDVerticalAlignmentThreshold = "vertical_alignment_threshold";
constexpr auto kPortIDAlignment = "alignment";

constexpr double kDefaultVerticalAlignmentThreshold = 0.7;  // Cosine of the largest tilt still called vertical.

inline constexpr auto kDescription = R"(
    <p>
        Classifies a grasp pose as <b>top</b> or <b>side</b> relative to a fitted
        object primitive.
    </p>
    <p>
        Compares the grasp pose's local Z axis against the primitive's local Z axis,
        both expressed in their shared header frame. The <b>primitive_pose</b> and
        <b>grasp_pose</b> must have matching <code>frame_id</code> values (e.g. both in
        <code>world</code>).
    </p>
    <ul>
        <li><b>top</b>: grasp Z parallel to primitive Z (approach aligned with the object's Z axis).</li>
        <li><b>side</b>: grasp Z perpendicular to primitive Z (approach normal to the object's Z axis).</li>
    </ul>
    <p>
        SPHERE primitives have no axis of symmetry; the behavior logs a warning and returns <b>top</b>.
    </p>
)";

}  // namespace

namespace grasping_behaviors
{

DeterminePickAlignment::DeterminePickAlignment(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList DeterminePickAlignment::providedPorts()
{
  return {
    BT::InputPort<shape_msgs::msg::SolidPrimitive>(kPortIDSolidPrimitive, "{solid_primitive}",
                                                   "Object primitive. Its local Z axis is the reference for 'top' vs "
                                                   "'side'."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPrimitivePose, "{primitive_pose}",
                                                   "Pose of the primitive in a shared frame. Must have the same "
                                                   "frame_id as grasp_pose."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        kPortIDGraspPose, "{grasp_pose}", "Grasp pose to classify. Must have the same frame_id as primitive_pose."),
    BT::InputPort<double>(kPortIDVerticalAlignmentThreshold, kDefaultVerticalAlignmentThreshold,
                          "Absolute cosine threshold between the grasp's Z axis and the primitive's Z axis. "
                          "Above this value the grasp is 'top'; at or below it is 'side'. Default 0.7 (~45 deg)."),
    BT::OutputPort<std::string>(kPortIDAlignment, "{alignment}",
                                "Either 'top' (grasp Z parallel to primitive Z) or 'side' (grasp Z perpendicular to "
                                "primitive Z)."),
  };
}

BT::KeyValueVector DeterminePickAlignment::metadata()
{
  return { { "subcategory", "Grasping" }, { "description", kDescription } };
}

BT::NodeStatus DeterminePickAlignment::tick()
{
  const auto solid_primitive_input = getInput<shape_msgs::msg::SolidPrimitive>(kPortIDSolidPrimitive);
  if (!solid_primitive_input.has_value())
  {
    spdlog::error("DeterminePickAlignment: failed to get 'solid_primitive' input port.");
    return BT::NodeStatus::FAILURE;
  }

  const auto primitive_pose_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPrimitivePose);
  if (!primitive_pose_input.has_value())
  {
    spdlog::error("DeterminePickAlignment: failed to get 'primitive_pose' input port.");
    return BT::NodeStatus::FAILURE;
  }

  const auto grasp_pose_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose);
  if (!grasp_pose_input.has_value())
  {
    spdlog::error("DeterminePickAlignment: failed to get 'grasp_pose' input port.");
    return BT::NodeStatus::FAILURE;
  }

  double threshold = kDefaultVerticalAlignmentThreshold;
  if (const auto threshold_input = getInput<double>(kPortIDVerticalAlignmentThreshold); threshold_input.has_value())
  {
    threshold = threshold_input.value();
  }

  const auto& primitive = solid_primitive_input.value();
  const auto& primitive_pose = primitive_pose_input.value();
  const auto& grasp_pose = grasp_pose_input.value();

  if (primitive_pose.header.frame_id != grasp_pose.header.frame_id)
  {
    spdlog::error("DeterminePickAlignment: primitive_pose frame '{}' does not match grasp_pose frame '{}'.",
                  primitive_pose.header.frame_id, grasp_pose.header.frame_id);
    return BT::NodeStatus::FAILURE;
  }

  if (primitive.type == shape_msgs::msg::SolidPrimitive::SPHERE)
  {
    spdlog::warn("DeterminePickAlignment: SPHERE primitive has no axis of symmetry; returning 'top'.");
    setOutput(kPortIDAlignment, std::string{ "top" });
    return BT::NodeStatus::SUCCESS;
  }

  const auto& primitive_orientation = primitive_pose.pose.orientation;
  const auto& grasp_orientation = grasp_pose.pose.orientation;

  const Eigen::Quaterniond q_primitive(primitive_orientation.w, primitive_orientation.x, primitive_orientation.y,
                                       primitive_orientation.z);
  const Eigen::Quaterniond q_grasp(grasp_orientation.w, grasp_orientation.x, grasp_orientation.y, grasp_orientation.z);

  if (q_primitive.norm() < 1e-6 || q_grasp.norm() < 1e-6)
  {
    spdlog::error("DeterminePickAlignment: a pose has a zero-norm orientation quaternion.");
    return BT::NodeStatus::FAILURE;
  }

  const Eigen::Vector3d primitive_z = q_primitive.normalized() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d grasp_z = q_grasp.normalized() * Eigen::Vector3d::UnitZ();

  const double alignment_cosine = std::abs(primitive_z.dot(grasp_z));

  const std::string alignment = (alignment_cosine > threshold) ? "top" : "side";

  spdlog::info("DeterminePickAlignment: |cos(grasp_z, primitive_z)| = {:.3f} (threshold {:.3f}) -> {} grasp",
               alignment_cosine, threshold, alignment);

  setOutput(kPortIDAlignment, alignment);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace grasping_behaviors
