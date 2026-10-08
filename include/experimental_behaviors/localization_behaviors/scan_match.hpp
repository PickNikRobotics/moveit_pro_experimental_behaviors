// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cstddef>
#include <vector>

namespace experimental_behaviors::localization_utils
{
/**
 * @brief Distance from any point on an occupancy grid's plane to the nearest occupied cell centre.
 * @details Built once per grid with an exact Euclidean distance transform. Unknown cells count as free.
 */
class OccupancyDistanceField
{
public:
  /** @param occupied_threshold Cell values at or above this are occupied, on the grid's 0 to 100 scale. */
  OccupancyDistanceField(const nav_msgs::msg::OccupancyGrid& grid, int occupied_threshold);

  /**
   * @brief Distance in metres from @p point_in_grid_frame to the nearest occupied cell.
   * @return +infinity when the point is off the grid or the grid has no occupied cell.
   */
  [[nodiscard]] double distance(const Eigen::Vector2d& point_in_grid_frame) const;

private:
  std::size_t width_;
  std::size_t height_;
  double resolution_;
  Eigen::Isometry2d grid_from_cells_;
  std::vector<double> distance_;
};

/** @brief How well a scan fits a map at one pose. */
struct ScanMatchScore
{
  double inlier_fraction;
  double median_residual;
  std::size_t beams_used;
};

/**
 * @brief Scores every usable beam of @p scan by its endpoint's distance to the nearest occupied cell.
 * @param scan_in_grid Pose of the scan's frame in the grid's frame, in 3D; only its planar part is used.
 * @details A beam is usable when its range is finite and inside the sensor's limits; a max-range return
 * has no endpoint. An endpoint off the grid is used, as an outlier.
 */
[[nodiscard]] ScanMatchScore scoreScan(const sensor_msgs::msg::LaserScan& scan, const Eigen::Isometry3d& scan_in_grid,
                                       const OccupancyDistanceField& field, double inlier_distance);
}  // namespace experimental_behaviors::localization_utils
