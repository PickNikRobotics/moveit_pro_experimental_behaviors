// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/pose_to_vectors.hpp>
#include <experimental_behaviors/vectors_to_pose.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace
{
// Helper: returns true if `haystack` contains `needle`.
bool contains(const std::string& haystack, const std::string& needle)
{
  return haystack.find(needle) != std::string::npos;
}
}  // namespace

// ---------------------------------------------------------------------------
// decomposePose
// ---------------------------------------------------------------------------

TEST(DecomposePose, IdentityPoseProducesZeroTranslationAndIdentityQuaternion)
{
  // GIVEN a default-constructed Pose (zero position, zero quaternion).
  geometry_msgs::msg::Pose pose;

  // WHEN decomposing.
  auto [translation, quaternion] = experimental_behaviors::detail::decomposePose(pose);

  // THEN both vectors should hold the default field values.
  ASSERT_EQ(translation.size(), 3u);
  ASSERT_EQ(quaternion.size(), 4u);
  EXPECT_DOUBLE_EQ(translation[0], 0.0);
  EXPECT_DOUBLE_EQ(translation[1], 0.0);
  EXPECT_DOUBLE_EQ(translation[2], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[0], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[1], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[2], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[3], 0.0);
}

TEST(DecomposePose, FilledPosePreservesFieldOrder)
{
  // GIVEN a Pose with distinct values in every field.
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.x = 0.1;
  pose.orientation.y = 0.2;
  pose.orientation.z = 0.3;
  pose.orientation.w = 0.9;

  // WHEN decomposing.
  auto [translation, quaternion] = experimental_behaviors::detail::decomposePose(pose);

  // THEN the vectors should be (x, y, z) and (x, y, z, w) in that order — adversarial
  // check against an off-by-one or x/y swap in the implementation.
  ASSERT_EQ(translation.size(), 3u);
  ASSERT_EQ(quaternion.size(), 4u);
  EXPECT_DOUBLE_EQ(translation[0], 1.0);
  EXPECT_DOUBLE_EQ(translation[1], 2.0);
  EXPECT_DOUBLE_EQ(translation[2], 3.0);
  EXPECT_DOUBLE_EQ(quaternion[0], 0.1);
  EXPECT_DOUBLE_EQ(quaternion[1], 0.2);
  EXPECT_DOUBLE_EQ(quaternion[2], 0.3);
  EXPECT_DOUBLE_EQ(quaternion[3], 0.9);
}

TEST(DecomposePose, NegativeAndExtremeValuesAreCopiedVerbatim)
{
  // GIVEN a Pose with negative and large values.
  geometry_msgs::msg::Pose pose;
  pose.position.x = -1.5;
  pose.position.y = 1e9;
  pose.position.z = -1e-9;
  pose.orientation.x = -1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 1.0;
  pose.orientation.w = -0.5;

  // WHEN decomposing.
  auto [translation, quaternion] = experimental_behaviors::detail::decomposePose(pose);

  // THEN values pass through unchanged (no normalization, no clamping).
  EXPECT_DOUBLE_EQ(translation[0], -1.5);
  EXPECT_DOUBLE_EQ(translation[1], 1e9);
  EXPECT_DOUBLE_EQ(translation[2], -1e-9);
  EXPECT_DOUBLE_EQ(quaternion[0], -1.0);
  EXPECT_DOUBLE_EQ(quaternion[1], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[2], 1.0);
  EXPECT_DOUBLE_EQ(quaternion[3], -0.5);
}

// ---------------------------------------------------------------------------
// composePose — error paths first per project test rules.
// ---------------------------------------------------------------------------

TEST(ComposePose, EmptyTranslationVectorReturnsError)
{
  // GIVEN an empty translation vector and a valid quaternion.
  const std::vector<double> translation;
  const std::vector<double> quaternion{ 0.0, 0.0, 0.0, 1.0 };

  // WHEN composing.
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN it should fail with a message that names the bad input and its size.
  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(contains(result.error(), "translation_xyz")) << result.error();
  EXPECT_TRUE(contains(result.error(), "3")) << result.error();
  EXPECT_TRUE(contains(result.error(), "0")) << result.error();
}

TEST(ComposePose, OversizedTranslationVectorReturnsError)
{
  // GIVEN a translation vector with too many elements.
  const std::vector<double> translation{ 1.0, 2.0, 3.0, 4.0 };
  const std::vector<double> quaternion{ 0.0, 0.0, 0.0, 1.0 };

  // WHEN composing.
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN error names the vector and its size.
  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(contains(result.error(), "translation_xyz")) << result.error();
  EXPECT_TRUE(contains(result.error(), "4")) << result.error();
}

TEST(ComposePose, EmptyQuaternionVectorReturnsError)
{
  // GIVEN a valid translation and an empty quaternion.
  const std::vector<double> translation{ 0.0, 0.0, 0.0 };
  const std::vector<double> quaternion;

  // WHEN composing.
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN error names the quaternion vector.
  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(contains(result.error(), "quaternion_xyzw")) << result.error();
  EXPECT_TRUE(contains(result.error(), "4")) << result.error();
}

TEST(ComposePose, OversizedQuaternionVectorReturnsError)
{
  // GIVEN a quaternion vector with too many elements.
  const std::vector<double> translation{ 0.0, 0.0, 0.0 };
  const std::vector<double> quaternion{ 0.0, 0.0, 0.0, 1.0, 0.0 };

  // WHEN composing.
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN error names the quaternion vector and its size.
  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(contains(result.error(), "quaternion_xyzw")) << result.error();
  EXPECT_TRUE(contains(result.error(), "5")) << result.error();
}

// ---------------------------------------------------------------------------
// composePose — happy path & ordering.
// ---------------------------------------------------------------------------

TEST(ComposePose, IdentityInputsProduceZeroPositionIdentityOrientation)
{
  // GIVEN canonical defaults: zero translation and identity quaternion (w=1).
  const std::vector<double> translation{ 0.0, 0.0, 0.0 };
  const std::vector<double> quaternion{ 0.0, 0.0, 0.0, 1.0 };

  // WHEN composing.
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN every field maps to the expected slot.
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value().position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.value().position.y, 0.0);
  EXPECT_DOUBLE_EQ(result.value().position.z, 0.0);
  EXPECT_DOUBLE_EQ(result.value().orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(result.value().orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(result.value().orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(result.value().orientation.w, 1.0);
}

TEST(ComposePose, DistinctInputsPreserveFieldOrder)
{
  // GIVEN distinct values in every slot — adversarial against x/y/z swaps or
  // off-by-one indexing in the quaternion.
  const std::vector<double> translation{ 7.0, 8.0, 9.0 };
  const std::vector<double> quaternion{ 0.5, 0.6, 0.7, 0.8 };

  // WHEN composing.
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN translation maps to position xyz and quaternion maps to orientation xyzw.
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value().position.x, 7.0);
  EXPECT_DOUBLE_EQ(result.value().position.y, 8.0);
  EXPECT_DOUBLE_EQ(result.value().position.z, 9.0);
  EXPECT_DOUBLE_EQ(result.value().orientation.x, 0.5);
  EXPECT_DOUBLE_EQ(result.value().orientation.y, 0.6);
  EXPECT_DOUBLE_EQ(result.value().orientation.z, 0.7);
  EXPECT_DOUBLE_EQ(result.value().orientation.w, 0.8);
}

// ---------------------------------------------------------------------------
// Round trip — decomposePose followed by composePose returns the original Pose.
// ---------------------------------------------------------------------------

TEST(DecomposeComposeRoundTrip, ArbitraryPoseSurvivesRoundTrip)
{
  // GIVEN an arbitrary pose.
  geometry_msgs::msg::Pose original;
  original.position.x = -3.14;
  original.position.y = 2.718;
  original.position.z = 0.0;
  original.orientation.x = 0.1;
  original.orientation.y = -0.2;
  original.orientation.z = 0.3;
  original.orientation.w = 0.9273618495495704;

  // WHEN we decompose and re-compose.
  auto [translation, quaternion] = experimental_behaviors::detail::decomposePose(original);
  const auto result = experimental_behaviors::detail::composePose(translation, quaternion);

  // THEN the round trip is exact (no precision loss in straight field copies).
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value().position.x, original.position.x);
  EXPECT_DOUBLE_EQ(result.value().position.y, original.position.y);
  EXPECT_DOUBLE_EQ(result.value().position.z, original.position.z);
  EXPECT_DOUBLE_EQ(result.value().orientation.x, original.orientation.x);
  EXPECT_DOUBLE_EQ(result.value().orientation.y, original.orientation.y);
  EXPECT_DOUBLE_EQ(result.value().orientation.z, original.orientation.z);
  EXPECT_DOUBLE_EQ(result.value().orientation.w, original.orientation.w);
}
