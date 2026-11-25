// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <Eigen/Core>
#include <sensor_msgs/msg/camera_info.hpp>

// Private to this folder's src/. The image folder has its own small copy of CameraIntrinsics,
// because a folder must not include another folder's headers.

namespace point_cloud_behaviors::camera_frustum
{
/** @brief Pinhole intrinsics and image size, read from a CameraInfo. */
struct CameraIntrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;

  /** @brief False when the intrinsics cannot project anything, as in a CameraInfo that was never filled. */
  [[nodiscard]] bool isUsable() const;
};

/** @brief Reads fx, fy, cx, cy from K and the image size from @p camera_info. */
[[nodiscard]] CameraIntrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo& camera_info);

/**
 * @brief The usable part of the camera view volume.
 * @details near_z and far_z are distances along the optical Z axis, not radial distances, so the
 * near and far caps are planes and the kept region is a true frustum.
 */
struct Frustum
{
  double near_z = 0.0;
  double far_z = 0.0;
  /** @brief Inset from all four image edges, in pixels. */
  double pixel_margin = 0.0;
};

/**
 * @brief True when @p point_optical, in camera optical coordinates, is inside @p frustum.
 * @details A point must also be in front of the camera. NaN coordinates are outside.
 */
[[nodiscard]] bool isInsideFrustum(const Eigen::Vector3d& point_optical, const CameraIntrinsics& intrinsics,
                                   const Frustum& frustum);
}  // namespace point_cloud_behaviors::camera_frustum
