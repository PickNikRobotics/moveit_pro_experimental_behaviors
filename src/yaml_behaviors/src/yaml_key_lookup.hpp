#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <tl_expected/expected.hpp>

namespace yaml_behaviors
{
/**
 * @brief Why a resolveKey() lookup failed.
 *
 * Callers that tolerate an unconfigured key must be able to tell that case
 * apart from a malformed one. Only kKeyNotFound says "this map is fine and
 * simply has no such key"; every other kind means the config is broken, and
 * treating those as "not configured" silently drops whatever the key controls.
 */
enum class KeyLookupErrorKind
{
  kKeyNotFound,     ///< Well-formed map, no such key. The only benign failure.
  kNodeNotAMap,     ///< Parent is undefined, null, or a non-map. Malformed config.
  kAmbiguousMatch,  ///< Case-insensitive mode matched two or more keys. Ambiguous config.
};

/// A resolveKey() failure: its kind, plus a message suitable for logging.
struct KeyLookupError
{
  KeyLookupErrorKind kind;
  std::string message;
};

/**
 * @brief Resolve a single child of @p map by @p key.
 *
 * Exact match always wins (checked first in both modes). When
 * @p case_sensitive is false and no exact key exists, the map is scanned for
 * ASCII case-insensitive matches: exactly one match is returned; zero matches
 * or two-or-more matches produce an error.
 *
 * @param map The YAML node to look up within (must be a map).
 * @param key The key to resolve.
 * @param case_sensitive When true, only an exact key match succeeds.
 * @return The resolved child node, or a KeyLookupError whose `kind`
 *         distinguishes an absent key from malformed or ambiguous input.
 */
tl::expected<YAML::Node, KeyLookupError> resolveKey(const YAML::Node& map, const std::string& key, bool case_sensitive);
}  // namespace yaml_behaviors
