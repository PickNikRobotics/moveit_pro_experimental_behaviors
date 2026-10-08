// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/localization_behaviors/scan_match.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// One pass of the Felzenszwalb-Huttenlocher squared distance transform, in place along a strided line.
void squaredDistanceTransform1D(std::vector<double>& data, std::size_t offset, std::size_t stride, std::size_t count)
{
  std::vector<double> f(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    f[i] = data[offset + i * stride];
  }
  std::vector<std::size_t> v(count);
  std::vector<double> z(count + 1);
  std::size_t k = 0;
  std::size_t first = count;
  for (std::size_t q = 0; q < count; ++q)
  {
    if (!std::isfinite(f[q]))
    {
      continue;
    }
    if (first == count)
    {
      first = q;
      v[0] = q;
      z[0] = -kInfinity;
      z[1] = kInfinity;
      continue;
    }
    double s = 0.0;
    while (true)
    {
      const auto p = static_cast<double>(v[k]);
      const auto qd = static_cast<double>(q);
      s = ((f[q] + qd * qd) - (f[v[k]] + p * p)) / (2.0 * (qd - p));
      if (s > z[k])
      {
        break;
      }
      --k;
    }
    // z[0] is -infinity, so s now lies right of z[k] and opens a new parabola.
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInfinity;
  }
  if (first == count)
  {
    return;  // No finite sample on this line: it stays infinite.
  }
  k = 0;
  for (std::size_t q = 0; q < count; ++q)
  {
    const auto qd = static_cast<double>(q);
    while (z[k + 1] < qd)
    {
      ++k;
    }
    const auto p = static_cast<double>(v[k]);
    data[offset + q * stride] = (qd - p) * (qd - p) + f[v[k]];
  }
}
}  // namespace

namespace experimental_behaviors::localization_utils
{
OccupancyDistanceField::OccupancyDistanceField(const nav_msgs::msg::OccupancyGrid& grid, int occupied_threshold)
  : width_{ grid.info.width }
  , height_{ grid.info.height }
  , resolution_{ grid.info.resolution }
  , distance_(static_cast<std::size_t>(grid.info.width) * grid.info.height, kInfinity)
{
  Eigen::Isometry3d origin;
  tf2::fromMsg(grid.info.origin, origin);
  const Eigen::Vector3d x_axis = origin.rotation().col(0);
  grid_from_cells_ =
      Eigen::Translation2d(origin.translation().head<2>()) * Eigen::Rotation2Dd(std::atan2(x_axis.y(), x_axis.x()));

  const std::size_t cells = std::min(distance_.size(), grid.data.size());
  for (std::size_t i = 0; i < cells; ++i)
  {
    if (grid.data[i] >= occupied_threshold)
    {
      distance_[i] = 0.0;
    }
  }
  for (std::size_t row = 0; row < height_; ++row)
  {
    squaredDistanceTransform1D(distance_, row * width_, 1, width_);
  }
  for (std::size_t column = 0; column < width_; ++column)
  {
    squaredDistanceTransform1D(distance_, column, width_, height_);
  }
  for (auto& value : distance_)
  {
    value = std::sqrt(value) * resolution_;
  }
}

double OccupancyDistanceField::distance(const Eigen::Vector2d& point_in_grid_frame) const
{
  const Eigen::Vector2d cell = grid_from_cells_.inverse() * point_in_grid_frame / resolution_;
  if (!(cell.x() >= 0.0 && cell.y() >= 0.0))
  {
    return kInfinity;
  }
  const auto column = static_cast<std::size_t>(cell.x());
  const auto row = static_cast<std::size_t>(cell.y());
  if (column >= width_ || row >= height_)
  {
    return kInfinity;
  }
  return distance_[row * width_ + column];
}

ScanMatchScore scoreScan(const sensor_msgs::msg::LaserScan& scan, const Eigen::Isometry3d& scan_in_grid,
                         const OccupancyDistanceField& field, double inlier_distance)
{
  std::vector<double> residuals;
  residuals.reserve(scan.ranges.size());
  std::size_t inliers = 0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i)
  {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max)
    {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const Eigen::Vector3d endpoint =
        scan_in_grid * Eigen::Vector3d{ range * std::cos(angle), range * std::sin(angle), 0.0 };
    const double residual = field.distance(endpoint.head<2>());
    residuals.push_back(residual);
    inliers += residual <= inlier_distance ? 1 : 0;
  }
  if (residuals.empty())
  {
    return { 0.0, kInfinity, 0 };
  }
  const auto middle = residuals.begin() + static_cast<std::ptrdiff_t>(residuals.size() / 2);
  std::nth_element(residuals.begin(), middle, residuals.end());
  return { static_cast<double>(inliers) / static_cast<double>(residuals.size()), *middle, residuals.size() };
}
}  // namespace experimental_behaviors::localization_utils
