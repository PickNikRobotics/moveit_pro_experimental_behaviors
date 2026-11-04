// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/joint_bounds_compatibility.hpp>
#include <experimental_behaviors/robot_state_compatibility.hpp>

#include <string>
#include <vector>

namespace
{
struct LegacyRobotState
{
  void setVariablePositions(const std::vector<std::string>& names, const std::vector<double>& positions)
  {
    names_ = names;
    positions_ = positions;
    data_ = positions.data();
  }

  std::vector<std::string> names_;
  std::vector<double> positions_;
  const double* data_ = nullptr;
};

struct CurrentRobotState
{
  void setVariablePositions(const std::vector<std::string>& names, const Eigen::Ref<const Eigen::VectorXd>& positions)
  {
    names_ = names;
    positions_ = positions;
    data_ = positions.data();
  }

  std::vector<std::string> names_;
  Eigen::VectorXd positions_;
  const double* data_ = nullptr;
};

TEST(RobotStateCompatibility, PreservesLegacyVectorInputWithoutCopying)
{
  LegacyRobotState state;
  const std::vector<std::string> names{ "shoulder_pan_joint", "elbow_joint" };
  const std::vector<double> positions{ 0.25, -1.5 };
  experimental_behaviors::detail::setVariablePositions(state, names, positions, 0);
  EXPECT_EQ(state.names_, names);
  EXPECT_EQ(state.positions_, positions);
  EXPECT_EQ(state.data_, positions.data());
}

TEST(RobotStateCompatibility, MapsCurrentEigenInputWithoutCopying)
{
  CurrentRobotState state;
  const std::vector<std::string> names{ "shoulder_pan_joint", "elbow_joint" };
  const std::vector<double> positions{ 0.25, -1.5 };
  experimental_behaviors::detail::setVariablePositions(state, names, positions, 0);
  EXPECT_EQ(state.names_, names);
  ASSERT_EQ(state.positions_.size(), 2);
  EXPECT_DOUBLE_EQ(state.positions_[0], positions[0]);
  EXPECT_DOUBLE_EQ(state.positions_[1], positions[1]);
  EXPECT_EQ(state.data_, positions.data());
}

TEST(RobotStateCompatibility, AcceptsEmptyLegacyVectors)
{
  LegacyRobotState state;
  experimental_behaviors::detail::setVariablePositions(state, {}, {}, 0);
  EXPECT_TRUE(state.names_.empty());
  EXPECT_TRUE(state.positions_.empty());
}

TEST(RobotStateCompatibility, AcceptsEmptyCurrentEigenViews)
{
  CurrentRobotState state;
  experimental_behaviors::detail::setVariablePositions(state, {}, {}, 0);
  EXPECT_TRUE(state.names_.empty());
  EXPECT_EQ(state.positions_.size(), 0);
}
struct LegacyBounds
{
  bool position_bounded_;
  double min_position_;
  double max_position_;
};
struct CurrentBounds
{
  bool position_bounded;
  double min_position;
  double max_position;
};
TEST(JointBoundsCompatibility, LegacyBoundsPreserveValues)
{
  // GIVEN bounded position limits in the legacy member spelling.
  const LegacyBounds source{ true, -1.5, 2.0 };
  // WHEN adapting the limits.
  const auto bounds = experimental_behaviors::detail::positionBounds(source, 0);
  // THEN every value retains its meaning.
  EXPECT_TRUE(bounds.bounded);
  EXPECT_DOUBLE_EQ(bounds.minimum, -1.5);
  EXPECT_DOUBLE_EQ(bounds.maximum, 2.0);
}
TEST(JointBoundsCompatibility, CurrentBoundsPreserveUnboundedFlag)
{
  // GIVEN current member spellings and an unbounded joint.
  const CurrentBounds source{ false, -1.5, 2.0 };
  // WHEN adapting the limits.
  const auto bounds = experimental_behaviors::detail::positionBounds(source, 0);
  // THEN the bounded flag and positions remain exact.
  EXPECT_FALSE(bounds.bounded);
  EXPECT_DOUBLE_EQ(bounds.minimum, -1.5);
  EXPECT_DOUBLE_EQ(bounds.maximum, 2.0);
}
}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
