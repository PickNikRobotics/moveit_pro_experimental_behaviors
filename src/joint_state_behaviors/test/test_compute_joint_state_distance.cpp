// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "joint_state_behaviors/compute_joint_state_distance.hpp"

using joint_state_behaviors::internal::jointStateDistance;

namespace
{
sensor_msgs::msg::JointState makeState(const std::vector<std::string>& names, const std::vector<double>& positions)
{
  sensor_msgs::msg::JointState state;
  state.name = names;
  state.position = positions;
  return state;
}
}  // namespace

TEST(ComputeJointStateDistance, EveryTargetJointWhenNoNamesGiven)
{
  // Names in a different order in the two states: the match is by name, not by index.
  const auto current = makeState({ "joint_b", "joint_a" }, { 0.0, 0.0 });
  const auto target = makeState({ "joint_a", "joint_b" }, { 3.0, -4.0 });
  const auto result = jointStateDistance(current, target, {});
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_DOUBLE_EQ(result->distance, 5.0);
  EXPECT_DOUBLE_EQ(result->max_joint_distance, 4.0);
}

TEST(ComputeJointStateDistance, OnlyTheNamedJoints)
{
  const auto current = makeState({ "joint_a", "joint_b", "linear_axis" }, { 1.0, 1.0, 0.0 });
  const auto target = makeState({ "joint_a", "joint_b", "linear_axis" }, { 1.0, 2.0, 7.0 });
  const auto arm = jointStateDistance(current, target, { "joint_a", "joint_b" });
  ASSERT_TRUE(arm.has_value()) << arm.error();
  EXPECT_DOUBLE_EQ(arm->distance, 1.0);
  const auto axis = jointStateDistance(current, target, { "linear_axis" });
  ASSERT_TRUE(axis.has_value()) << axis.error();
  EXPECT_DOUBLE_EQ(axis->distance, 7.0);
  EXPECT_DOUBLE_EQ(axis->max_joint_distance, 7.0);
}

TEST(ComputeJointStateDistance, MissingJointFails)
{
  const auto current = makeState({ "joint_a" }, { 0.0 });
  const auto target = makeState({ "joint_a", "joint_b" }, { 0.0, 1.0 });
  EXPECT_FALSE(jointStateDistance(current, target, {}).has_value());
  EXPECT_FALSE(jointStateDistance(target, current, { "joint_b" }).has_value());
}

TEST(ComputeJointStateDistance, JointWithoutPositionFails)
{
  auto current = makeState({ "joint_a", "joint_b" }, { 0.0, 0.0 });
  current.position.resize(1);
  const auto target = makeState({ "joint_a", "joint_b" }, { 0.0, 1.0 });
  EXPECT_FALSE(jointStateDistance(current, target, {}).has_value());
}

TEST(ComputeJointStateDistance, NonFinitePositionFails)
{
  const auto current = makeState({ "joint_a" }, { std::numeric_limits<double>::quiet_NaN() });
  const auto target = makeState({ "joint_a" }, { 0.0 });
  EXPECT_FALSE(jointStateDistance(current, target, {}).has_value());
}

TEST(ComputeJointStateDistance, NothingToCompareFails)
{
  const auto empty = makeState({}, {});
  EXPECT_FALSE(jointStateDistance(empty, empty, {}).has_value());
  EXPECT_FALSE(jointStateDistance(empty, empty, { "" }).has_value());
}
