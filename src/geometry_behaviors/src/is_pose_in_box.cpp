#include "geometry_behaviors/is_pose_in_box.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr auto kPortIDPose = "pose";
constexpr auto kPortIDBoxCentroidPose = "box_centroid_pose";
constexpr auto kPortIDBoxSize = "box_size";
constexpr auto kPortIDIsPoseInBox = "is_pose_in_box";

const auto kTfLookupTimeoutSeconds = std::chrono::duration<double>{ 0.5 };
}  // namespace

namespace geometry_behaviors
{

IsPoseInBox::IsPoseInBox(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList IsPoseInBox::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPose, "{pose}", "The pose to check."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDBoxCentroidPose, "{box_centroid_pose}",
                                                       "The box to check that the pose is in."),
        BT::InputPort<std::vector<double>>(kPortIDBoxSize, "{box_size}",
                                           "The X, Y, and Z dimensions of the region of interest."),
        BT::OutputPort<bool>(kPortIDIsPoseInBox, "{is_pose_in_box}", "Is the pose inside of the box?") });
}

BT::KeyValueVector IsPoseInBox::metadata()
{
  return { { "description", "Calculates if a pose is inside a box." }, { "subcategory", "Utility" } };
}

BT::NodeStatus IsPoseInBox::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDPose),
                                               getInput<geometry_msgs::msg::PoseStamped>(kPortIDBoxCentroidPose),
                                               getInput<std::vector<double>>(kPortIDBoxSize));

  if (!ports.has_value())
  {
    spdlog::warn("IsPoseInBox: missing required input: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose, box_centroid_pose, box_size] = ports.value();

  if (box_size.size() != 3)
  {
    spdlog::warn(std::string("The value for input data port ")
                     .append(kPortIDBoxSize)
                     .append(" must be a vector containing exactly 3 elements. Got a vector containing ")
                     .append(std::to_string(box_size.size()).append(" elements.")));
    return BT::NodeStatus::FAILURE;
  }

  const Eigen::Vector3d box_dimensions{ box_size.at(0), box_size.at(1), box_size.at(2) };

  // Get the transform from box reference frame to box centroid.
  Eigen::Isometry3d tform_box_reference_frame_to_box_centroid;
  tf2::fromMsg(box_centroid_pose.pose, tform_box_reference_frame_to_box_centroid);

  // Get transform from box reference frame to pose frame.
  Eigen::Isometry3d tform_box_reference_frame_to_pose_frame;
  try
  {
    const auto tform = getBehaviorContext()->transform_buffer_ptr->lookupTransform(
        box_centroid_pose.header.frame_id, pose.header.frame_id, tf2_ros::fromMsg(pose.header.stamp),
        tf2::durationFromSec(kTfLookupTimeoutSeconds.count()));
    tform_box_reference_frame_to_pose_frame = tf2::transformToEigen(tform);
  }
  catch (const std::exception& e)
  {
    spdlog::warn("Failed to get transform from crop box reference frame to pose frame: " + std::string(e.what()));
    return BT::NodeStatus::FAILURE;
  }

  // Transform pose to box reference frame.
  Eigen::Isometry3d pose_in_pose_frame;
  tf2::fromMsg(pose.pose, pose_in_pose_frame);

  const Eigen::Isometry3d pose_in_box_frame = tform_box_reference_frame_to_pose_frame * pose_in_pose_frame;

  // Get position relative to box centroid.
  const Eigen::Vector3d& pose_position_in_box_frame = pose_in_box_frame.translation();
  const Eigen::Vector3d& box_centroid_position = tform_box_reference_frame_to_box_centroid.translation();
  const Eigen::Vector3d relative_position = pose_position_in_box_frame - box_centroid_position;

  // Check if pose is within the box bounds.
  const bool is_within_box = std::abs(relative_position.x()) <= box_dimensions.x() / 2.0 &&
                             std::abs(relative_position.y()) <= box_dimensions.y() / 2.0 &&
                             std::abs(relative_position.z()) <= box_dimensions.z() / 2.0;

  setOutput(kPortIDIsPoseInBox, is_within_box);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
