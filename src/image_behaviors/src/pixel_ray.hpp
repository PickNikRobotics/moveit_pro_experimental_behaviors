// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <Eigen/Core>
#include <sensor_msgs/msg/camera_info.hpp>

#include <optional>

// Private to this folder's src/. The point cloud folder has its own small copy of CameraIntrinsics,
// because a folder must not include another folder's headers.

namespace image_behaviors::pixel_ray
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
 * @brief Unit ray direction, in camera optical coordinates, through a pixel given as fractions of the image size.
 * @details Fractions of the image width and height are what GetCenterFromMask2D outputs. Lens
 * distortion is not removed: the ray is exact only for a rectified image.
 */
[[nodiscard]] Eigen::Vector3d rayDirectionFromNormalizedPixel(double u_norm, double v_norm,
                                                              const CameraIntrinsics& intrinsics);

/**
 * @brief Point where the ray from @p origin along @p direction meets the plane through @p plane_point with normal
 * @p plane_normal.
 * @return nullopt when the ray is parallel to the plane or meets it behind @p origin.
 */
[[nodiscard]] std::optional<Eigen::Vector3d> intersectRayWithPlane(const Eigen::Vector3d& origin,
                                                                   const Eigen::Vector3d& direction,
                                                                   const Eigen::Vector3d& plane_point,
                                                                   const Eigen::Vector3d& plane_normal);
}  // namespace image_behaviors::pixel_ray
