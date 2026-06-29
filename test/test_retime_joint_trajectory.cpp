// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <experimental_behaviors/retime_joint_trajectory.hpp>

namespace
{
using Indices = std::vector<std::size_t>;

// Evenly spaced timestamps [0, 1, 2, ..., n-1] seconds.
std::vector<double> makeTimes(std::size_t n)
{
  std::vector<double> times;
  times.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    times.push_back(static_cast<double>(i));
  }
  return times;
}
}  // namespace

namespace experimental_behaviors
{
TEST(SelectRetimeWaypointIndices, EmptyInputReturnsEmpty)
{
  // GIVEN an empty time vector
  const std::vector<double> times;

  // WHEN selecting indices
  const auto kept = selectRetimeWaypointIndices(times, 1.0, 1.0, 3);

  // THEN no indices are returned
  EXPECT_TRUE(kept.empty());
}

TEST(SelectRetimeWaypointIndices, StrideOfOneKeepsEveryWaypoint)
{
  // GIVEN 5 evenly spaced points and no preserve windows
  const auto times = makeTimes(5);

  // WHEN the middle stride is 1 (keep all)
  const auto kept = selectRetimeWaypointIndices(times, 0.0, 0.0, 1);

  // THEN every index is kept
  EXPECT_EQ(kept, (Indices{ 0, 1, 2, 3, 4 }));
}

TEST(SelectRetimeWaypointIndices, StrideBelowOneIsTreatedAsOne)
{
  // GIVEN 4 points and an invalid stride of 0
  const auto times = makeTimes(4);

  // WHEN selecting with keep_every_nth = 0
  const auto kept = selectRetimeWaypointIndices(times, 0.0, 0.0, 0);

  // THEN it behaves like stride 1 and keeps everything
  EXPECT_EQ(kept, (Indices{ 0, 1, 2, 3 }));
}

TEST(SelectRetimeWaypointIndices, MiddleThinnedByStrideWithFirstAndLastImplicitlyPreserved)
{
  // GIVEN 10 points at t = 0..9 and zero preserve windows.
  // The first point (t=0 <= 0) is in the head and the last (t=9 >= 9) is in the tail,
  // so the middle is indices 1..8.
  const auto times = makeTimes(10);

  // WHEN keeping every 3rd middle point
  const auto kept = selectRetimeWaypointIndices(times, 0.0, 0.0, 3);

  // THEN head (0), every 3rd middle point (1, 4, 7), and tail (9) are kept
  EXPECT_EQ(kept, (Indices{ 0, 1, 4, 7, 9 }));
}

TEST(SelectRetimeWaypointIndices, PreserveWindowsKeepHeadAndTailVerbatim)
{
  // GIVEN 10 points at t = 0..9
  const auto times = makeTimes(10);

  // WHEN preserving the first 2s and last 2s, thinning the middle aggressively
  const auto kept = selectRetimeWaypointIndices(times, 2.0, 2.0, 100);

  // THEN head (t<=2 -> 0,1,2), tail (t>=7 -> 7,8,9), and the first middle point (index 3) are kept.
  // Middle indices are 3,4,5,6; with stride 100 only the first (index 3) survives.
  EXPECT_EQ(kept, (Indices{ 0, 1, 2, 3, 7, 8, 9 }));
}

TEST(SelectRetimeWaypointIndices, PreserveWindowBoundaryIsInclusive)
{
  // GIVEN points at t = 0..9 and a 3s start-preserve window.
  const auto times = makeTimes(10);

  // WHEN start_preserve_seconds is exactly 3.0
  const auto kept = selectRetimeWaypointIndices(times, 3.0, 0.0, 100);

  // THEN the point at exactly t=3 is preserved (inclusive boundary), so head is 0,1,2,3.
  // Middle is 4..8 (t=9 is the tail); stride 100 keeps only the first middle point (index 4).
  EXPECT_EQ(kept, (Indices{ 0, 1, 2, 3, 4, 9 }));
}

TEST(SelectRetimeWaypointIndices, NegativePreserveSecondsClampToZero)
{
  // GIVEN 5 points and negative preserve windows
  const auto times = makeTimes(5);

  // WHEN preserve windows are negative
  const auto kept = selectRetimeWaypointIndices(times, -5.0, -5.0, 2);

  // THEN they clamp to 0: head is index 0, tail is index 4, middle (1,2,3) keeps every 2nd (1, 3)
  EXPECT_EQ(kept, (Indices{ 0, 1, 3, 4 }));
}

TEST(SelectRetimeWaypointIndices, UntimedTrajectoryKeepsAllWaypoints)
{
  // GIVEN a trajectory whose timestamps are all zero (timing stripped)
  const std::vector<double> times(6, 0.0);

  // WHEN selecting with a preserve window and a stride
  const auto kept = selectRetimeWaypointIndices(times, 1.0, 1.0, 3);

  // THEN total duration is 0, the tail threshold is <= 0, every point lands in the tail,
  // and nothing is dropped.
  EXPECT_EQ(kept, (Indices{ 0, 1, 2, 3, 4, 5 }));
}

TEST(SelectRetimeWaypointIndices, StrideTwoOnLongMiddleProbesOffByOne)
{
  // GIVEN 8 points at t = 0..7, head = index 0, tail = index 7, middle = 1..6.
  const auto times = makeTimes(8);

  // WHEN keeping every 2nd middle point
  const auto kept = selectRetimeWaypointIndices(times, 0.0, 0.0, 2);

  // THEN middle counter 0,2,4 (indices 1,3,5) survive; 2,4,6 are dropped.
  // A '<' vs '<=' or stride+1 bug would shift this result.
  EXPECT_EQ(kept, (Indices{ 0, 1, 3, 5, 7 }));
}
}  // namespace experimental_behaviors

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
