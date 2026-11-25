// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

// Pure unit tests for AddColoredVirtualObjectToPlanningScene::buildRequest, a static helper with no ROS
// dependency, so no executor, factory or service mock. The cases covered are in docs/AGENT_COMMENTS.md.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "planning_scene_behaviors/add_colored_virtual_object_to_planning_scene.hpp"

namespace planning_scene_behaviors
{
namespace
{

using ::testing::HasSubstr;

constexpr double kR = 0.1;
constexpr double kG = 0.2;
constexpr double kB = 0.3;
constexpr double kA = 0.4;

// Minimal CollisionObject — id only. The helper does not inspect geometry; it only forwards
// the message into the request unchanged (after setting operation = ADD and possibly
// prefixing the id).
moveit_msgs::msg::CollisionObject makeCollisionObject(const std::string& id)
{
  moveit_msgs::msg::CollisionObject co;
  co.id = id;
  co.header.frame_id = "world";
  return co;
}

// Returns the index of `id` in `entry_names`, or -1 if absent. Helpers that hand-roll a
// linear search keep the assertions readable.
int indexOf(const std::vector<std::string>& entry_names, const std::string& id)
{
  const auto it = std::find(entry_names.begin(), entry_names.end(), id);
  if (it == entry_names.end())
  {
    return -1;
  }
  return static_cast<int>(std::distance(entry_names.begin(), it));
}

// Build a snapshot whose ACM already names the listed ids. All cross-collisions are
// default-denied; rows are square (entry_values[i].enabled.size() == entry_names.size()).
moveit_msgs::msg::PlanningScene snapshotWithAcmEntries(const std::vector<std::string>& ids)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.allowed_collision_matrix.entry_names = ids;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    moveit_msgs::msg::AllowedCollisionEntry row;
    row.enabled = std::vector<bool>(ids.size(), false);
    row.enabled[i] = true;  // Self-collision allowed.
    scene.allowed_collision_matrix.entry_values.push_back(row);
  }
  return scene;
}

// ---------- Error paths: RGBA validation ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, NegativeChannelReturnsError)
{
  // GIVEN a valid object + scene but a negative blue channel
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("box"), moveit_msgs::msg::PlanningScene{}, kR, kG, -0.01, kA);

  // WHEN the helper runs
  // THEN it fails with a message naming the out-of-range value
  ASSERT_FALSE(result.has_value());
  EXPECT_THAT(result.error(), HasSubstr("out of range"));
}

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, ChannelAboveOneReturnsError)
{
  // GIVEN red above 1.0
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("box"), moveit_msgs::msg::PlanningScene{}, 1.01, kG, kB, kA);

  // WHEN the helper runs
  // THEN it fails
  ASSERT_FALSE(result.has_value());
  EXPECT_THAT(result.error(), HasSubstr("out of range"));
}

// ---------- Boundary values: 0.0 / 1.0 accepted ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, UnitRangeBoundariesAccepted)
{
  // GIVEN exact-boundary values 0.0 and 1.0
  // WHEN the helper runs
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("box"), moveit_msgs::msg::PlanningScene{}, 0.0, 1.0, 0.0, 1.0);

  // THEN it succeeds and forwards the values verbatim into the ObjectColor
  ASSERT_TRUE(result.has_value());
  ASSERT_THAT(result->scene.object_colors, testing::SizeIs(1));
  EXPECT_FLOAT_EQ(result->scene.object_colors[0].color.r, 0.0F);
  EXPECT_FLOAT_EQ(result->scene.object_colors[0].color.g, 1.0F);
  EXPECT_FLOAT_EQ(result->scene.object_colors[0].color.b, 0.0F);
  EXPECT_FLOAT_EQ(result->scene.object_colors[0].color.a, 1.0F);
}

// ---------- Prefix handling ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, IdWithoutPrefixGetsPrefixed)
{
  // GIVEN an unprefixed id
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("box"), moveit_msgs::msg::PlanningScene{}, kR, kG, kB, kA);

  // WHEN the helper runs
  // THEN the resulting collision_object.id starts with "virtual_"
  ASSERT_TRUE(result.has_value());
  ASSERT_THAT(result->scene.world.collision_objects, testing::SizeIs(1));
  EXPECT_EQ(result->scene.world.collision_objects[0].id, "virtual_box");
  EXPECT_EQ(result->scene.object_colors[0].id, "virtual_box");
}

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, IdAlreadyPrefixedNotDoublePrefixed)
{
  // GIVEN an id that already starts with "virtual_"
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("virtual_box"), moveit_msgs::msg::PlanningScene{}, kR, kG, kB, kA);

  // WHEN the helper runs
  // THEN the id stays as-is — no "virtual_virtual_" double-prefix
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->scene.world.collision_objects[0].id, "virtual_box");
}

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, RegressionIdContainingPrefixMidStringStillGetsPrefixed)
{
  // GIVEN an id that contains "virtual_" as a substring but does NOT start with it.
  //       Under the previous std::string::find(prefix) == npos check, this id was
  //       wrongly classified as already-prefixed and skipped — leaving an unprefixed id
  //       in the planning scene that did not match the ACM virtual-obstacle convention.
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("my_virtual_box"), moveit_msgs::msg::PlanningScene{}, kR, kG, kB, kA);

  // WHEN the helper runs
  // THEN it prefixes the id (mutation: flipping starts_with back to a substring search
  //      would make this test fail).
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->scene.world.collision_objects[0].id, "virtual_my_virtual_box");
}

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, EmptyIdBecomesPrefixOnly)
{
  // GIVEN an empty id (degenerate but legal)
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject(""), moveit_msgs::msg::PlanningScene{}, kR, kG, kB, kA);

  // WHEN the helper runs
  // THEN the id is just "virtual_"
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->scene.world.collision_objects[0].id, "virtual_");
}

// ---------- Operation overwrite ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, OperationOverwrittenToAdd)
{
  // GIVEN a CollisionObject with operation = REMOVE (caller might forget to set it)
  auto co = makeCollisionObject("box");
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  // WHEN the helper runs
  const auto result =
      AddColoredVirtualObjectToPlanningScene::buildRequest(co, moveit_msgs::msg::PlanningScene{}, kR, kG, kB, kA);

  // THEN the request forces operation = ADD — otherwise we'd silently delete the obstacle
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->scene.world.collision_objects[0].operation, moveit_msgs::msg::CollisionObject::ADD);
}

// ---------- ACM construction: clean snapshot ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, CleanSceneProducesSingleSelfAllowedDefaultDeniedRow)
{
  // GIVEN an empty planning scene
  const auto result = AddColoredVirtualObjectToPlanningScene::buildRequest(
      makeCollisionObject("box"), moveit_msgs::msg::PlanningScene{}, kR, kG, kB, kA);

  // WHEN the helper runs
  ASSERT_TRUE(result.has_value());
  const auto& acm = result->scene.allowed_collision_matrix;

  // THEN the ACM has exactly one row/column, named "virtual_box"
  ASSERT_THAT(acm.entry_names, testing::ElementsAre("virtual_box"));
  ASSERT_THAT(acm.entry_values, testing::SizeIs(1));
  ASSERT_THAT(acm.entry_values[0].enabled, testing::ElementsAre(true));  // Self-allowed.

  // AND the default-entries forbid collision with everything else
  ASSERT_THAT(acm.default_entry_names, testing::ElementsAre("virtual_box"));
  ASSERT_THAT(acm.default_entry_values, testing::ElementsAre(false));

  // AND the request is a diff
  EXPECT_TRUE(result->scene.is_diff);
}

// ---------- ACM construction: existing entries ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, ExistingVirtualObstacleAllowedBothDirections)
{
  // GIVEN a snapshot whose ACM contains another virtual obstacle
  auto scene = snapshotWithAcmEntries({ "virtual_table" });

  // WHEN the helper adds a new colored virtual obstacle
  const auto result =
      AddColoredVirtualObjectToPlanningScene::buildRequest(makeCollisionObject("box"), scene, kR, kG, kB, kA);

  // THEN the new and existing virtual obstacles are mutually allowed in the ACM
  ASSERT_TRUE(result.has_value());
  const auto& acm = result->scene.allowed_collision_matrix;
  const int new_idx = indexOf(acm.entry_names, "virtual_box");
  const int old_idx = indexOf(acm.entry_names, "virtual_table");
  ASSERT_GE(new_idx, 0);
  ASSERT_GE(old_idx, 0);
  EXPECT_TRUE(acm.entry_values.at(new_idx).enabled.at(old_idx)) << "new -> existing virtual must be allowed";
  EXPECT_TRUE(acm.entry_values.at(old_idx).enabled.at(new_idx)) << "existing virtual -> new must be allowed";
}

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, ExistingNonVirtualObstacleForbiddenBothDirections)
{
  // GIVEN a snapshot whose ACM contains a real (non-virtual) obstacle
  auto scene = snapshotWithAcmEntries({ "real_box" });

  // WHEN the helper adds a new colored virtual obstacle
  const auto result =
      AddColoredVirtualObjectToPlanningScene::buildRequest(makeCollisionObject("box"), scene, kR, kG, kB, kA);

  // THEN the new virtual obstacle and the real obstacle are mutually forbidden
  ASSERT_TRUE(result.has_value());
  const auto& acm = result->scene.allowed_collision_matrix;
  const int new_idx = indexOf(acm.entry_names, "virtual_box");
  const int old_idx = indexOf(acm.entry_names, "real_box");
  ASSERT_GE(new_idx, 0);
  ASSERT_GE(old_idx, 0);
  EXPECT_FALSE(acm.entry_values.at(new_idx).enabled.at(old_idx)) << "new -> real must be forbidden";
  EXPECT_FALSE(acm.entry_values.at(old_idx).enabled.at(new_idx)) << "real -> new must be forbidden";
}

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest,
     RegressionExistingIdContainingPrefixMidStringTreatedAsNonVirtual)
{
  // GIVEN a snapshot id with "virtual_" in the middle, which is a non-virtual obstacle. An older substring
  //       check called it virtual and allowed it to collide with the new obstacle.
  auto scene = snapshotWithAcmEntries({ "my_virtual_box" });

  // WHEN the helper adds a new colored virtual obstacle
  const auto result =
      AddColoredVirtualObjectToPlanningScene::buildRequest(makeCollisionObject("box"), scene, kR, kG, kB, kA);

  // THEN the new virtual obstacle and the misnamed real obstacle are FORBIDDEN
  //      (mutation: flipping the per-existing-entry check back to substring would
  //      make this test fail).
  ASSERT_TRUE(result.has_value());
  const auto& acm = result->scene.allowed_collision_matrix;
  const int new_idx = indexOf(acm.entry_names, "virtual_box");
  const int old_idx = indexOf(acm.entry_names, "my_virtual_box");
  ASSERT_GE(new_idx, 0);
  ASSERT_GE(old_idx, 0);
  EXPECT_FALSE(acm.entry_values.at(new_idx).enabled.at(old_idx));
  EXPECT_FALSE(acm.entry_values.at(old_idx).enabled.at(new_idx));
}

// ---------- Duplicate-id diagnostic contract ----------

TEST(AddColoredVirtualObjectToPlanningSceneBuildRequest, DuplicateIdInSnapshotStillProducesRequest)
{
  // GIVEN a snapshot whose ACM already contains the prefixed id we're about to add
  auto scene = snapshotWithAcmEntries({ "virtual_box" });

  // WHEN the helper runs
  const auto result =
      AddColoredVirtualObjectToPlanningScene::buildRequest(makeCollisionObject("box"), scene, kR, kG, kB, kA);

  // THEN it does NOT no-op — the contract is "warn but proceed" (a future change to
  //      no-op or replace would need to update this test). The resulting ACM has two
  //      "virtual_box" entries; downstream consumers see duplicates, but the helper's
  //      job is to be loud about it (via spdlog::warn), not silently drop the request.
  ASSERT_TRUE(result.has_value());
  const auto& names = result->scene.allowed_collision_matrix.entry_names;
  EXPECT_EQ(std::count(names.begin(), names.end(), std::string{ "virtual_box" }), 2);
}

}  // namespace
}  // namespace planning_scene_behaviors
