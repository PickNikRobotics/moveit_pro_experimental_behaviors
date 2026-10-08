// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include <experimental_behaviors/localization_behaviors/scan_match.hpp>

namespace
{
using experimental_behaviors::localization_utils::OccupancyDistanceField;
using experimental_behaviors::localization_utils::scoreScan;

constexpr int kOccupied = 65;
constexpr double kResolution = 0.1;

// A square room: occupied border cells, free inside.
nav_msgs::msg::OccupancyGrid makeRoom(std::uint32_t cells)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = kResolution;
  grid.info.width = cells;
  grid.info.height = cells;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(cells) * cells, 0);
  for (std::uint32_t i = 0; i < cells; ++i)
  {
    grid.data[i] = 100;
    grid.data[(cells - 1) * cells + i] = 100;
    grid.data[i * cells] = 100;
    grid.data[i * cells + cells - 1] = 100;
  }
  return grid;
}

// One beam straight ahead along the scan frame's +X.
sensor_msgs::msg::LaserScan makeScan(std::vector<float> ranges, double angle_increment = 0.0)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.0F;
  scan.angle_increment = static_cast<float>(angle_increment);
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges = std::move(ranges);
  return scan;
}

Eigen::Isometry3d planarPose(double x, double y, double yaw)
{
  return Eigen::Translation3d(x, y, 0.0) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}
}  // namespace

TEST(OccupancyDistanceField, MatchesBruteForceOnARandomGrid)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = kResolution;
  grid.info.width = 23;
  grid.info.height = 17;
  grid.info.origin.orientation.w = 1.0;
  std::mt19937 rng{ 7 };
  std::uniform_int_distribution<int> value{ -1, 100 };
  for (std::size_t i = 0; i < 23 * 17; ++i)
  {
    grid.data.push_back(static_cast<std::int8_t>(value(rng) > 90 ? 100 : 0));
  }
  const OccupancyDistanceField field{ grid, kOccupied };
  for (int row = 0; row < 17; ++row)
  {
    for (int column = 0; column < 23; ++column)
    {
      double best = std::numeric_limits<double>::infinity();
      for (int r = 0; r < 17; ++r)
      {
        for (int c = 0; c < 23; ++c)
        {
          if (grid.data[r * 23 + c] >= kOccupied)
          {
            best = std::min(best, std::hypot(r - row, c - column) * kResolution);
          }
        }
      }
      const Eigen::Vector2d centre{ (column + 0.5) * kResolution, (row + 0.5) * kResolution };
      EXPECT_NEAR(field.distance(centre), best, 1e-6) << "cell " << column << "," << row;
    }
  }
}

TEST(OccupancyDistanceField, OffGridAndEmptyGridAreInfinite)
{
  const OccupancyDistanceField room{ makeRoom(10), kOccupied };
  EXPECT_TRUE(std::isinf(room.distance({ -0.01, 0.5 })));
  EXPECT_TRUE(std::isinf(room.distance({ 0.5, 1.01 })));

  auto empty = makeRoom(10);
  std::fill(empty.data.begin(), empty.data.end(), 0);
  EXPECT_TRUE(std::isinf(OccupancyDistanceField{ empty, kOccupied }.distance({ 0.5, 0.5 })));
}

TEST(OccupancyDistanceField, HonoursARotatedOrigin)
{
  auto grid = makeRoom(10);
  // Grid x axis along map +Y, so the grid's first column lies along map y = 0.
  grid.info.origin.orientation.z = std::sin(M_PI / 4.0);
  grid.info.origin.orientation.w = std::cos(M_PI / 4.0);
  const OccupancyDistanceField field{ grid, kOccupied };
  EXPECT_NEAR(field.distance({ -0.05, 0.05 }), 0.0, 1e-6);
  EXPECT_TRUE(std::isinf(field.distance({ 0.05, 0.05 })));
}

TEST(OccupancyDistanceField, UnknownCellsCountAsFree)
{
  auto grid = makeRoom(10);
  std::fill(grid.data.begin(), grid.data.end(), -1);
  grid.data[0] = 100;
  const OccupancyDistanceField field{ grid, kOccupied };
  // The nearest occupied cell is the corner one, not an unknown neighbour.
  EXPECT_NEAR(field.distance({ 2.5 * kResolution, 0.5 * kResolution }), 2.0 * kResolution, 1e-6);
}

TEST(OccupancyDistanceField, HonoursATranslatedOrigin)
{
  auto grid = makeRoom(10);
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = 2.0;
  const OccupancyDistanceField field{ grid, kOccupied };
  EXPECT_NEAR(field.distance({ -1.0 + 0.5 * kResolution, 2.0 + 0.5 * kResolution }), 0.0, 1e-6);
  EXPECT_TRUE(std::isinf(field.distance({ 0.05, 0.05 })));
}

TEST(OccupancyDistanceField, ShortDataLeavesTheRestFree)
{
  auto grid = makeRoom(10);
  // Only the first row arrives; the missing cells must not be read.
  grid.data.resize(10);
  const OccupancyDistanceField field{ grid, kOccupied };
  EXPECT_NEAR(field.distance({ 0.05, 0.05 }), 0.0, 1e-6);
  EXPECT_NEAR(field.distance({ 0.05, 0.35 }), 3.0 * kResolution, 1e-6);
}

TEST(ScoreScan, AllBeamsOnTheWallAtTheTruePose)
{
  const OccupancyDistanceField field{ makeRoom(20), kOccupied };
  // From the room centre, each beam ends inside the first wall cell it reaches.
  const auto score = scoreScan(makeScan({ 0.95F, 0.95F, 0.95F }, M_PI / 2.0), planarPose(1.0, 1.0, 0.0), field, 0.05);
  EXPECT_EQ(score.beams_used, 3U);
  EXPECT_DOUBLE_EQ(score.inlier_fraction, 1.0);
  EXPECT_NEAR(score.median_residual, 0.0, 1e-6);
}

TEST(ScoreScan, AShiftedPoseLosesItsInliers)
{
  const OccupancyDistanceField field{ makeRoom(20), kOccupied };
  const auto score = scoreScan(makeScan({ 0.95F }), planarPose(0.7, 1.0, 0.0), field, 0.05);
  EXPECT_EQ(score.beams_used, 1U);
  EXPECT_DOUBLE_EQ(score.inlier_fraction, 0.0);
  EXPECT_NEAR(score.median_residual, 0.3, 1e-6);
}

TEST(ScoreScan, SkipsInvalidAndMaxRangeBeams)
{
  const OccupancyDistanceField field{ makeRoom(20), kOccupied };
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto score = scoreScan(makeScan({ 0.95F, nan, 0.01F, 10.0F, std::numeric_limits<float>::infinity() }),
                               planarPose(1.0, 1.0, 0.0), field, 0.05);
  EXPECT_EQ(score.beams_used, 1U);
  EXPECT_DOUBLE_EQ(score.inlier_fraction, 1.0);
}

TEST(ScoreScan, NoUsableBeam)
{
  const OccupancyDistanceField field{ makeRoom(20), kOccupied };
  const auto score = scoreScan(makeScan({ 10.0F }), planarPose(1.0, 1.0, 0.0), field, 0.05);
  EXPECT_EQ(score.beams_used, 0U);
  EXPECT_DOUBLE_EQ(score.inlier_fraction, 0.0);
}

TEST(ScoreScan, OffMapEndpointIsAnOutlier)
{
  const OccupancyDistanceField field{ makeRoom(20), kOccupied };
  const auto score = scoreScan(makeScan({ 0.95F, 5.0F }, M_PI), planarPose(1.0, 1.0, 0.0), field, 0.05);
  EXPECT_EQ(score.beams_used, 2U);
  EXPECT_DOUBLE_EQ(score.inlier_fraction, 0.5);
}

TEST(ScoreScan, UsesTheSensorOffset)
{
  const OccupancyDistanceField field{ makeRoom(20), kOccupied };
  // Robot at the centre, sensor mounted forward of it: the same range only fits once the offset is applied.
  const Eigen::Isometry3d robot = planarPose(1.0, 1.0, 0.0);
  const Eigen::Isometry3d sensor_in_robot = planarPose(0.4, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(scoreScan(makeScan({ 0.55F }), robot * sensor_in_robot, field, 0.05).inlier_fraction, 1.0);
  EXPECT_DOUBLE_EQ(scoreScan(makeScan({ 0.55F }), robot, field, 0.05).inlier_fraction, 0.0);
}
