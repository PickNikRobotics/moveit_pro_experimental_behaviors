// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <cmath>

#include "grasping_behaviors/generate_place_candidates.hpp"

using grasping_behaviors::internal::generatePlaceCandidates;
using grasping_behaviors::internal::PlaceCandidateRequest;

namespace
{
// Object picked from above: grasp link straight above the object, pointing down.
PlaceCandidateRequest topGraspRequest()
{
  PlaceCandidateRequest request;
  request.object_at_pick = Eigen::Translation3d(1.0, 0.0, 0.5) * Eigen::Isometry3d::Identity();
  request.grasp_at_pick = Eigen::Translation3d(1.0, 0.0, 0.6) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                          Eigen::Isometry3d::Identity();
  request.target = Eigen::Translation3d(0.0, 2.0, 0.8) * Eigen::Isometry3d::Identity();
  return request;
}
}  // namespace

TEST(GeneratePlaceCandidates, NominalCandidateKeepsTheGraspTransform)
{
  auto request = topGraspRequest();
  request.target_offset = Eigen::Vector3d(0.0, 0.0, 0.05);
  const auto result = generatePlaceCandidates(request);
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_EQ(result->grasp_poses.size(), 1u);

  const Eigen::Isometry3d expected_object = Eigen::Translation3d(0.0, 2.0, 0.85) * Eigen::Isometry3d::Identity();
  EXPECT_TRUE(result->predicted_object_pose.isApprox(expected_object, 1e-12));
  // The grasp link lands where it was relative to the object at pick time.
  const Eigen::Isometry3d grasp_to_object = request.grasp_at_pick.inverse() * request.object_at_pick;
  EXPECT_TRUE((result->grasp_poses[0] * grasp_to_object).isApprox(expected_object, 1e-12));
  EXPECT_NEAR(result->grasp_poses[0].translation().z(), 0.95, 1e-12);
}

TEST(GeneratePlaceCandidates, YawWindowIsNominalFirstAndSymmetric)
{
  auto request = topGraspRequest();
  request.yaw_range_deg = 20.0;
  request.yaw_increment_deg = 10.0;
  const auto result = generatePlaceCandidates(request);
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_EQ(result->grasp_poses.size(), 5u);

  const auto yaw_of = [](const Eigen::Isometry3d& pose) {
    const Eigen::Vector3d x_axis = pose.linear().col(0);
    return std::atan2(x_axis.y(), x_axis.x()) * 180.0 / M_PI;
  };
  EXPECT_NEAR(yaw_of(result->grasp_poses[0]), 0.0, 1e-9);
  EXPECT_NEAR(yaw_of(result->grasp_poses[1]), -10.0, 1e-9);
  EXPECT_NEAR(yaw_of(result->grasp_poses[2]), 10.0, 1e-9);
  EXPECT_NEAR(yaw_of(result->grasp_poses[3]), -20.0, 1e-9);
  EXPECT_NEAR(yaw_of(result->grasp_poses[4]), 20.0, 1e-9);
  // Every turn is about the vertical axis through the place point, so the grasp stays above it.
  for (const auto& pose : result->grasp_poses)
  {
    EXPECT_NEAR(pose.translation().x(), 0.0, 1e-9);
    EXPECT_NEAR(pose.translation().y(), 2.0, 1e-9);
  }
}

TEST(GeneratePlaceCandidates, FlippedCandidatesComeAfterUpright)
{
  auto request = topGraspRequest();
  request.include_flipped = true;
  const auto result = generatePlaceCandidates(request);
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_EQ(result->grasp_poses.size(), 2u);
  // Upside down, the grasp link is below the object origin instead of above it.
  EXPECT_GT(result->grasp_poses[0].translation().z(), 0.8);
  EXPECT_LT(result->grasp_poses[1].translation().z(), 0.8);
}

TEST(GeneratePlaceCandidates, InvalidRequestsFail)
{
  auto none = topGraspRequest();
  none.include_upright = false;
  EXPECT_FALSE(generatePlaceCandidates(none).has_value());

  auto bad_step = topGraspRequest();
  bad_step.yaw_range_deg = 10.0;
  bad_step.yaw_increment_deg = 0.0;
  EXPECT_FALSE(generatePlaceCandidates(bad_step).has_value());
}
