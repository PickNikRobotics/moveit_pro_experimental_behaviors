// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "pixel_ray.hpp"

#include <cmath>

namespace image_behaviors::pixel_ray
{
namespace
{
// Below this, the ray is so close to parallel to the plane that the intersection is meaningless.
constexpr double kParallelEpsilon = 1e-9;
}  // namespace

bool CameraIntrinsics::isUsable() const
{
  return fx > 0.0 && fy > 0.0 && width > 0 && height > 0;
}

CameraIntrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo& camera_info)
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = camera_info.k[0];
  intrinsics.fy = camera_info.k[4];
  intrinsics.cx = camera_info.k[2];
  intrinsics.cy = camera_info.k[5];
  intrinsics.width = static_cast<int>(camera_info.width);
  intrinsics.height = static_cast<int>(camera_info.height);
  return intrinsics;
}

Eigen::Vector3d rayDirectionFromNormalizedPixel(const double u_norm, const double v_norm,
                                                const CameraIntrinsics& intrinsics)
{
  const double u = u_norm * static_cast<double>(intrinsics.width);
  const double v = v_norm * static_cast<double>(intrinsics.height);
  const Eigen::Vector3d direction((u - intrinsics.cx) / intrinsics.fx, (v - intrinsics.cy) / intrinsics.fy, 1.0);
  return direction.normalized();
}

std::optional<Eigen::Vector3d> intersectRayWithPlane(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                                                     const Eigen::Vector3d& plane_point,
                                                     const Eigen::Vector3d& plane_normal)
{
  const double denominator = plane_normal.dot(direction);
  if (std::abs(denominator) < kParallelEpsilon)
  {
    return std::nullopt;
  }
  // A ray that points away from the plane still meets the infinite plane, behind the origin; reject it.
  const double t = plane_normal.dot(plane_point - origin) / denominator;
  if (!(t > 0.0))
  {
    return std::nullopt;
  }
  return origin + t * direction;
}
}  // namespace image_behaviors::pixel_ray
