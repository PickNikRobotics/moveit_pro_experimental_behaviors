// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "joint_state_behaviors/is_joint_position_in_range.hpp"

#include <limits>

using joint_state_behaviors::internal::jointPosition;

namespace
{
sensor_msgs::msg::JointState state()
{
  sensor_msgs::msg::JointState s;
  s.name = { "left_joint1", "left_robotiq_85_left_knuckle_joint", "right_robotiq_85_left_knuckle_joint" };
  s.position = { 0.3, 0.46, 0.80 };
  return s;
}
}  // namespace

TEST(IsJointPositionInRange, FindsTheNamedJoint)
{
  const auto p = jointPosition(state(), "left_robotiq_85_left_knuckle_joint");
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(*p, 0.46);
}

TEST(IsJointPositionInRange, MissingJointFails)
{
  EXPECT_FALSE(jointPosition(state(), "no_such_joint").has_value());
}

TEST(IsJointPositionInRange, JointWithoutPositionFails)
{
  auto s = state();
  s.position.resize(1);
  EXPECT_FALSE(jointPosition(s, "right_robotiq_85_left_knuckle_joint").has_value());
}

TEST(IsJointPositionInRange, NonFinitePositionFails)
{
  auto s = state();
  s.position[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(jointPosition(s, "left_robotiq_85_left_knuckle_joint").has_value());
}
