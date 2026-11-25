// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "camera_frustum.hpp"

namespace point_cloud_behaviors::camera_frustum
{
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

bool isInsideFrustum(const Eigen::Vector3d& point_optical, const CameraIntrinsics& intrinsics, const Frustum& frustum)
{
  if (!intrinsics.isUsable())
  {
    return false;
  }

  // The negated comparisons also reject NaN, which is how an invalid depth pixel arrives.
  const double z = point_optical.z();
  if (!(z > 0.0) || !(z >= frustum.near_z) || !(z <= frustum.far_z))
  {
    return false;
  }

  const double u = intrinsics.fx * point_optical.x() / z + intrinsics.cx;
  const double v = intrinsics.fy * point_optical.y() / z + intrinsics.cy;
  const double margin = frustum.pixel_margin;
  return u >= margin && u <= static_cast<double>(intrinsics.width) - 1.0 - margin && v >= margin &&
         v <= static_cast<double>(intrinsics.height) - 1.0 - margin;
}
}  // namespace point_cloud_behaviors::camera_frustum
