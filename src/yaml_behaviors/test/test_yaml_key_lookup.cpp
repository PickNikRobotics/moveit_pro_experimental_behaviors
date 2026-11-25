#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "yaml_key_lookup.hpp"

using yaml_behaviors::KeyLookupErrorKind;
using yaml_behaviors::resolveKey;

namespace
{
YAML::Node makeMap()
{
  // WAX1_P2 is the real-world key shape we care about.
  return YAML::Load("{WAX1_P2: hello, RINSE: world}");
}
}  // namespace

TEST(ResolveKey, ExactMatchCaseSensitive)
{
  auto r = resolveKey(makeMap(), "WAX1_P2", /*case_sensitive=*/true);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r.value().as<std::string>(), "hello");
}

TEST(ResolveKey, CaseMismatchFailsWhenSensitive)
{
  auto r = resolveKey(makeMap(), "wax1_p2", /*case_sensitive=*/true);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, KeyLookupErrorKind::kKeyNotFound);
}

TEST(ResolveKey, CaseMismatchHitsWhenInsensitive)
{
  auto r = resolveKey(makeMap(), "wax1_p2", /*case_sensitive=*/false);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r.value().as<std::string>(), "hello");
}

TEST(ResolveKey, ExactWinsOverFuzzyWhenInsensitive)
{
  // Both an exact and a differently-cased key exist; exact must win, no error.
  YAML::Node m = YAML::Load("{wax1: lower, WAX1: upper}");
  auto r = resolveKey(m, "wax1", /*case_sensitive=*/false);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r.value().as<std::string>(), "lower");
}

TEST(ResolveKey, AmbiguousMatchIsError)
{
  // Neither is an exact match for the target, but two fold to the same.
  YAML::Node m = YAML::Load("{WAX1: upper, wax1: lower}");
  auto r = resolveKey(m, "Wax1", /*case_sensitive=*/false);
  ASSERT_FALSE(r.has_value());
  EXPECT_NE(r.error().message.find("WAX1"), std::string::npos);
  EXPECT_NE(r.error().message.find("wax1"), std::string::npos);
}

TEST(ResolveKey, NotFoundInsensitive)
{
  auto r = resolveKey(makeMap(), "nope", /*case_sensitive=*/false);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, KeyLookupErrorKind::kKeyNotFound);
}

TEST(ResolveKey, NonMapNodeIsError)
{
  YAML::Node scalar = YAML::Load("just_a_scalar");
  auto r = resolveKey(scalar, "anything", /*case_sensitive=*/true);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, KeyLookupErrorKind::kNodeNotAMap);
}

TEST(ResolveKey, UndefinedNodeIsError)
{
  YAML::Node undefined;  // default-constructed: not defined
  auto r = resolveKey(undefined, "anything", /*case_sensitive=*/true);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, KeyLookupErrorKind::kNodeNotAMap);
}

// Only kKeyNotFound may mean "not configured" for ReadYamlValue's `optional` port. A malformed or
// ambiguous node that reported it would make an optional read silently return no value.
TEST(ResolveKey, OnlyAbsentKeysReportKeyNotFound)
{
  YAML::Node ambiguous = YAML::Load("{WAX1: upper, wax1: lower}");
  EXPECT_NE(resolveKey(ambiguous, "Wax1", /*case_sensitive=*/false).error().kind, KeyLookupErrorKind::kKeyNotFound);

  YAML::Node sequence = YAML::Load("[a, b]");
  EXPECT_NE(resolveKey(sequence, "anything", /*case_sensitive=*/true).error().kind, KeyLookupErrorKind::kKeyNotFound);

  YAML::Node null_node = YAML::Load("~");
  EXPECT_NE(resolveKey(null_node, "anything", /*case_sensitive=*/true).error().kind, KeyLookupErrorKind::kKeyNotFound);

  // ...while a well-formed map that simply lacks the key does report it.
  EXPECT_EQ(resolveKey(makeMap(), "absent", /*case_sensitive=*/true).error().kind, KeyLookupErrorKind::kKeyNotFound);
}
