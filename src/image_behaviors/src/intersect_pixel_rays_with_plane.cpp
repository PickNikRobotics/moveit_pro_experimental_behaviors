// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "image_behaviors/intersect_pixel_rays_with_plane.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "pixel_ray.hpp"

namespace
{
constexpr auto kPortCameraInfo = "camera_info";
constexpr auto kPortPixelCoords = "pixel_coords";
constexpr auto kPortPlanePose = "plane_pose";
constexpr auto kPortOutputPoses = "output_poses";
constexpr auto kPortHitCount = "hit_count";
constexpr auto kPortReferencePixel = "reference_pixel";
constexpr auto kPortReferencePose = "reference_pose";

// Below this norm the plane_pose orientation is not a rotation, so it has no normal.
constexpr double kMinQuaternionNorm = 1e-9;
}  // namespace

namespace image_behaviors
{
IntersectPixelRaysWithPlane::IntersectPixelRaysWithPlane(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList IntersectPixelRaysWithPlane::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortCameraInfo, "{camera_info}",
                                                "Camera the pixels were measured in. Its frame_id is where the rays "
                                                "start. The image must be rectified."),
    BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortPixelCoords, "{pixel_coords}",
                                                                 "Pixels as fractions of the image width (x) and "
                                                                 "height (y), as GetCenterFromMask2D outputs them."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPlanePose, "{plane_pose}",
                                                   "Plane to project onto: its position is on the plane and its Z axis "
                                                   "is the plane normal."),
    BT::InputPort<std::vector<double>>(kPortReferencePixel, "",
                                       "Optional extra pixel as \"u;v\", in the same fractional units, projected with "
                                       "the others. Use it to get a "
                                       "reference point on the plane. Empty means none."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        kPortOutputPoses, "{output_poses}",
        "Intersections in the frame of plane_pose, each with the orientation of plane_pose. Rays that miss are "
        "dropped, so this can be shorter than pixel_coords and is not index-aligned with it."),
    BT::OutputPort<int>(kPortHitCount, "{hit_count}", "Number of pixels whose ray met the plane."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortReferencePose, "{reference_pose}",
                                                    "Where reference_pixel met the plane. Not set when reference_pixel "
                                                    "is empty."),
  };
}

BT::KeyValueVector IntersectPixelRaysWithPlane::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - 2D Image" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Projects pixels onto a plane by intersecting their camera rays with it, instead of reading a depth "
             "value at each pixel. On a handle, a label or a thin or shiny feature the depth is that of the feature "
             "or it is missing; the ray meets the plane where the plane is." } };
}

BT::NodeStatus IntersectPixelRaysWithPlane::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::CameraInfo>(kPortCameraInfo),
      getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortPixelCoords),
      getInput<geometry_msgs::msg::PoseStamped>(kPortPlanePose));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [camera_info, pixel_coords, plane_pose] = ports.value();

  if (pixel_coords.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "pixel_coords is empty; there is nothing to project.");
    return BT::NodeStatus::FAILURE;
  }

  const auto intrinsics = pixel_ray::intrinsicsFromCameraInfo(camera_info);
  if (!intrinsics.isUsable())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "camera_info has no usable intrinsics; it was probably never filled.");
    return BT::NodeStatus::FAILURE;
  }

  Eigen::Quaterniond plane_orientation(plane_pose.pose.orientation.w, plane_pose.pose.orientation.x,
                                       plane_pose.pose.orientation.y, plane_pose.pose.orientation.z);
  if (!(plane_orientation.norm() > kMinQuaternionNorm))
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "plane_pose has no valid orientation.");
    return BT::NodeStatus::FAILURE;
  }
  plane_orientation.normalize();
  const Eigen::Vector3d plane_normal = plane_orientation.toRotationMatrix().col(2);
  const Eigen::Vector3d plane_point(plane_pose.pose.position.x, plane_pose.pose.position.y, plane_pose.pose.position.z);

  Eigen::Isometry3d optical_to_plane = Eigen::Isometry3d::Identity();
  if (camera_info.header.frame_id != plane_pose.header.frame_id)
  {
    try
    {
      optical_to_plane = tf2::transformToEigen(getBehaviorContext()->transform_buffer_ptr->lookupTransform(
          plane_pose.header.frame_id, camera_info.header.frame_id, tf2::TimePointZero));
    }
    catch (const std::exception& exception)
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), "Cannot transform the camera into the frame of the plane: " + std::string(exception.what()));
      return BT::NodeStatus::FAILURE;
    }
  }
  const Eigen::Vector3d ray_origin = optical_to_plane.translation();
  const Eigen::Matrix3d ray_rotation = optical_to_plane.rotation();

  const auto project = [&](const double u_norm, const double v_norm) {
    const Eigen::Vector3d direction =
        ray_rotation * pixel_ray::rayDirectionFromNormalizedPixel(u_norm, v_norm, intrinsics);
    return pixel_ray::intersectRayWithPlane(ray_origin, direction, plane_point, plane_normal);
  };
  const auto make_pose = [&](const Eigen::Vector3d& hit) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = plane_pose.header.frame_id;
    pose.header.stamp = getBehaviorContext()->node->now();
    pose.pose.position.x = hit.x();
    pose.pose.position.y = hit.y();
    pose.pose.position.z = hit.z();
    pose.pose.orientation = plane_pose.pose.orientation;
    return pose;
  };

  std::vector<geometry_msgs::msg::PoseStamped> output_poses;
  output_poses.reserve(pixel_coords.size());
  for (const auto& pixel : pixel_coords)
  {
    if (const auto hit = project(pixel.point.x, pixel.point.y); hit.has_value())
    {
      output_poses.push_back(make_pose(hit.value()));
    }
  }

  if (output_poses.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "None of the " + std::to_string(pixel_coords.size()) +
                    " pixel rays met the plane. The camera probably does not face it.");
    return BT::NodeStatus::FAILURE;
  }
  if (output_poses.size() < pixel_coords.size())
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), std::to_string(pixel_coords.size() - output_poses.size()) + " of " +
                    std::to_string(pixel_coords.size()) + " pixel rays did not meet the plane and were dropped.");
  }

  const auto reference_pixel = getInput<std::vector<double>>(kPortReferencePixel).value_or(std::vector<double>{});
  if (!reference_pixel.empty())
  {
    if (reference_pixel.size() != 2)
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(),
          "reference_pixel needs exactly two values, \"u;v\", but has " + std::to_string(reference_pixel.size()) + ".");
      return BT::NodeStatus::FAILURE;
    }
    const auto hit = project(reference_pixel[0], reference_pixel[1]);
    if (!hit.has_value())
    {
      getBehaviorContext()->logger->publishFailureMessage(name(),
                                                          "The ray of reference_pixel does not meet the plane.");
      return BT::NodeStatus::FAILURE;
    }
    setOutput(kPortReferencePose, make_pose(hit.value()));
  }

  setOutput(kPortOutputPoses, output_poses);
  setOutput(kPortHitCount, static_cast<int>(output_poses.size()));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace image_behaviors
