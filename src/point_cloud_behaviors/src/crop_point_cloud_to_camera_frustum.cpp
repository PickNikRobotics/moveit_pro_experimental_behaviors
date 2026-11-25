// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "point_cloud_behaviors/crop_point_cloud_to_camera_frustum.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include <string>

#include "camera_frustum.hpp"
#include "point_cloud_xyz.hpp"

namespace
{
constexpr auto kPortPointCloud = "point_cloud";
constexpr auto kPortCameraInfo = "camera_info";
constexpr auto kPortMinRange = "min_range";
constexpr auto kPortMaxRange = "max_range";
constexpr auto kPortPixelMargin = "pixel_margin";
constexpr auto kPortPointCloudCropped = "point_cloud_cropped";
constexpr auto kPortKeptCount = "kept_count";

// Wide defaults that keep the whole useful view of a typical depth camera; narrow them per camera.
constexpr double kDefaultMinRange = 0.0;
constexpr double kDefaultMaxRange = 10.0;
constexpr double kDefaultPixelMargin = 0.0;
}  // namespace

namespace point_cloud_behaviors
{
CropPointCloudToCameraFrustum::CropPointCloudToCameraFrustum(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList CropPointCloudToCameraFrustum::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortPointCloud, "{point_cloud}",
                                                 "Input point cloud with FLOAT32 x, y and z fields."),
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortCameraInfo, "{camera_info}",
                                                "Camera whose view volume defines the crop. Its frame_id is the "
                                                "optical frame the test is done in; the "
                                                "cloud is transformed into it when the two frames differ."),
    BT::InputPort<double>(kPortMinRange, kDefaultMinRange,
                          "Near limit, in meters along the optical Z axis. Closer points are dropped."),
    BT::InputPort<double>(kPortMaxRange, kDefaultMaxRange,
                          "Far limit, in meters along the optical Z axis. Further points are dropped."),
    BT::InputPort<double>(kPortPixelMargin, kDefaultPixelMargin,
                          "Inset from all four image edges, in pixels. Depth at the image edge is often the least "
                          "reliable."),
    BT::OutputPort<sensor_msgs::msg::PointCloud2>(kPortPointCloudCropped, "{point_cloud_cropped}",
                                                  "The kept points, in the frame of the input cloud. Unordered, with "
                                                  "every field kept."),
    BT::OutputPort<int>(kPortKeptCount, "{kept_count}", "Number of points kept."),
  };
}

BT::KeyValueVector CropPointCloudToCameraFrustum::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - 3D Point Cloud" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Keeps only the points inside the view volume of a camera: between a near and a far limit along the "
             "optical Z axis, and inside the image bounds. The limits are planes, not spheres, so the kept region "
             "is a true frustum. The output is unordered." } };
}

BT::NodeStatus CropPointCloudToCameraFrustum::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::PointCloud2>(kPortPointCloud),
                                                              getInput<sensor_msgs::msg::CameraInfo>(kPortCameraInfo));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [cloud, camera_info] = ports.value();

  const auto intrinsics = camera_frustum::intrinsicsFromCameraInfo(camera_info);
  if (!intrinsics.isUsable())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "camera_info has no usable intrinsics; it was probably never filled.");
    return BT::NodeStatus::FAILURE;
  }

  camera_frustum::Frustum frustum;
  frustum.near_z = getInput<double>(kPortMinRange).value_or(kDefaultMinRange);
  frustum.far_z = getInput<double>(kPortMaxRange).value_or(kDefaultMaxRange);
  frustum.pixel_margin = getInput<double>(kPortPixelMargin).value_or(kDefaultPixelMargin);
  if (!(frustum.near_z < frustum.far_z) || !(frustum.pixel_margin >= 0.0))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "min_range must be below max_range, and pixel_margin must not be negative.");
    return BT::NodeStatus::FAILURE;
  }

  const auto layout = point_cloud_xyz::findXyzLayout(cloud);
  if (!layout.has_value() || !point_cloud_xyz::hasConsistentSize(cloud))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "point_cloud has no little-endian FLOAT32 x, y and z fields, or its data is shorter than its size.");
    return BT::NodeStatus::FAILURE;
  }

  // Test in the camera optical frame, but keep the points as they are in their own frame, so this
  // stays a filter and never changes the frame of a cloud.
  Eigen::Isometry3d cloud_to_optical = Eigen::Isometry3d::Identity();
  if (cloud.header.frame_id != camera_info.header.frame_id)
  {
    try
    {
      cloud_to_optical = tf2::transformToEigen(getBehaviorContext()->transform_buffer_ptr->lookupTransform(
          camera_info.header.frame_id, cloud.header.frame_id, tf2::TimePointZero));
    }
    catch (const std::exception& exception)
    {
      getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot transform the cloud into the camera frame: " +
                                                                      std::string(exception.what()));
      return BT::NodeStatus::FAILURE;
    }
  }

  auto cropped = point_cloud_xyz::makeUnorderedLike(cloud);
  // Every kept point is finite, because the frustum test rejects NaN.
  cropped.is_dense = true;
  for (std::uint32_t row = 0; row < cloud.height; ++row)
  {
    for (std::uint32_t column = 0; column < cloud.width; ++column)
    {
      const auto* data = point_cloud_xyz::pointData(cloud, row, column);
      const Eigen::Vector3d point = point_cloud_xyz::readPoint(data, layout.value());
      if (camera_frustum::isInsideFrustum(cloud_to_optical * point, intrinsics, frustum))
      {
        point_cloud_xyz::appendPoint(cropped, data);
      }
    }
  }

  setOutput(kPortPointCloudCropped, cropped);
  setOutput(kPortKeptCount, static_cast<int>(cropped.width));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace point_cloud_behaviors
