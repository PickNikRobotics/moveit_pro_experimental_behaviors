// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

// Internal to the YAML behaviors — deliberately not under include/, because
// nothing outside this package needs it.

#include <string>
#include <vector>

namespace experimental_behaviors
{

/// Renders a key path as "key1.key2..." for error messages.
inline std::string keyPathToString(const std::vector<std::string>& keys)
{
  std::string joined;
  for (const auto& key : keys)
  {
    if (!joined.empty())
    {
      joined += '.';
    }
    joined += key;
  }
  return joined;
}

}  // namespace experimental_behaviors
