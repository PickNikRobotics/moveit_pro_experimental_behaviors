// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>

#include <string>
#include <vector>

namespace experimental_behaviors::test
{
TEST(JsonUtilsTest, PointerRoundTrip)
{
  const auto tokens = json_utils::parsePointer("/a~1b/~0/0/-");
  ASSERT_TRUE(tokens.has_value());
  EXPECT_EQ(tokens.value(), (std::vector<std::string>{ "a/b", "~", "0", "-" }));
  EXPECT_EQ(json_utils::pointerToString(tokens.value(), tokens->size()), "/a~1b/~0/0/-");
  EXPECT_TRUE(json_utils::parsePointer("")->empty());
  EXPECT_EQ(json_utils::parsePointer("/")->size(), 1u);
  EXPECT_FALSE(json_utils::parsePointer("no-slash").has_value());
}

TEST(JsonUtilsTest, SetFindRemove)
{
  json_utils::Json doc = json_utils::Json::object();
  ASSERT_TRUE(json_utils::set(doc, { "a", "b" }, 1, true).has_value());
  ASSERT_TRUE(json_utils::set(doc, { "list" }, json_utils::Json::array(), true).has_value());
  ASSERT_TRUE(json_utils::set(doc, { "list", "-", "x" }, true, true).has_value());
  EXPECT_EQ(doc, json_utils::Json::parse(R"({"a": {"b": 1}, "list": [{"x": true}]})"));

  const auto found = json_utils::find(doc, { "list", "0", "x" });
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(**found, true);

  EXPECT_EQ(json_utils::remove(doc, { "a", "b" }).value(), true);
  EXPECT_EQ(json_utils::remove(doc, { "a", "b" }).value(), false);
  EXPECT_EQ(json_utils::remove(doc, { "list", "-" }).value(), false);
}

TEST(JsonUtilsTest, NullParentIsReplacedOnlyWithCreateMissing)
{
  auto doc = json_utils::Json::parse(R"({"a": null})");
  EXPECT_FALSE(json_utils::set(doc, { "a", "b" }, 1, false).has_value());
  ASSERT_TRUE(json_utils::set(doc, { "a", "b" }, 1, true).has_value());
  EXPECT_EQ(doc, json_utils::Json::parse(R"({"a": {"b": 1}})"));
}

TEST(JsonUtilsTest, MissingParentIsAlwaysAnObject)
{
  json_utils::Json doc = json_utils::Json::object();
  ASSERT_TRUE(json_utils::set(doc, { "2024", "0" }, "x", true).has_value());
  EXPECT_EQ(doc, json_utils::Json::parse(R"({"2024": {"0": "x"}})"));
}
}  // namespace experimental_behaviors::test
