#include "yaml_key_lookup.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace yaml_behaviors
{
namespace
{
std::string toLowerAscii(const std::string& s)
{
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}
}  // namespace

tl::expected<YAML::Node, KeyLookupError> resolveKey(const YAML::Node& map, const std::string& key, bool case_sensitive)
{
  if (!map.IsDefined() || map.IsNull())
  {
    return tl::make_unexpected(
        KeyLookupError{ KeyLookupErrorKind::kNodeNotAMap, "YAML node is undefined when looking up key '" + key + "'" });
  }
  if (!map.IsMap())
  {
    return tl::make_unexpected(
        KeyLookupError{ KeyLookupErrorKind::kNodeNotAMap, "YAML node at key '" + key + "' is not a map" });
  }

  // Exact match always wins, in both modes.
  if (map[key])
  {
    return map[key];
  }

  if (case_sensitive)
  {
    return tl::make_unexpected(
        KeyLookupError{ KeyLookupErrorKind::kKeyNotFound, "Key '" + key + "' not found in YAML file" });
  }

  // Case-insensitive fallback: collect every key that folds to the target.
  const std::string key_lower = toLowerAscii(key);
  std::vector<std::string> matched_keys;
  for (const auto& entry : map)
  {
    if (!entry.first.IsScalar())
    {
      continue;
    }
    const std::string candidate = entry.first.as<std::string>();
    if (toLowerAscii(candidate) == key_lower)
    {
      matched_keys.push_back(candidate);
    }
  }

  if (matched_keys.empty())
  {
    return tl::make_unexpected(KeyLookupError{ KeyLookupErrorKind::kKeyNotFound,
                                               "Key '" + key + "' not found in YAML file (case-insensitive)" });
  }
  if (matched_keys.size() > 1)
  {
    std::string joined;
    for (size_t i = 0; i < matched_keys.size(); ++i)
    {
      joined += (i ? ", '" : "'") + matched_keys[i] + "'";
    }
    return tl::make_unexpected(KeyLookupError{
        KeyLookupErrorKind::kAmbiguousMatch, "Ambiguous case-insensitive match for '" + key + "': matches " + joined });
  }

  return map[matched_keys.front()];
}
}  // namespace yaml_behaviors
