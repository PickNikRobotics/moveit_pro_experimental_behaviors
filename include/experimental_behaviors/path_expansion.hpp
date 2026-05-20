// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <wordexp.h>

#include <string>
#include <string_view>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace experimental_behaviors
{

/**
 * @brief Resolves a path string to an absolute filesystem path.
 *
 * Accepts three forms:
 *
 *   1. **Plain absolute path** — e.g. `/mnt/nvme/autowash/foo.yaml`.
 *      Returned unchanged.
 *   2. **Absolute path with shell-style expansion** — `${VAR}` and
 *      leading `~` are expanded against the calling process's
 *      environment (via `wordexp(3)` with `WRDE_NOCMD` so command
 *      substitution is disabled).
 *   3. **ROS package URL** — `package://<package_name>/<rest>` resolves
 *      `<package_name>` via
 *      `ament_index_cpp::get_package_share_directory` and prepends the
 *      result to `<rest>` (mirrors the convention used by URDF
 *      `<mesh filename="package://...">`).
 *
 * Returns the resolved absolute path. On any expansion failure (unknown
 * package, malformed env var, etc.) falls back to returning the literal
 * input; downstream `std::filesystem::exists` / `YAML::LoadFile` calls
 * surface the resulting failure to the caller with a useful error.
 */
inline std::string expandPath(const std::string& in)
{
  // Form 3: package://<pkg>/<rest>
  static constexpr std::string_view kPackagePrefix = "package://";
  if (in.compare(0, kPackagePrefix.size(), kPackagePrefix) == 0)
  {
    const std::string rest = in.substr(kPackagePrefix.size());
    const auto slash = rest.find('/');
    const std::string pkg = (slash == std::string::npos) ? rest : rest.substr(0, slash);
    const std::string sub = (slash == std::string::npos) ? "" : rest.substr(slash);
    try
    {
      const std::string share = ament_index_cpp::get_package_share_directory(pkg);
      return share + sub;
    }
    catch (const std::exception&)
    {
      return in;  // literal fallback — caller surfaces the failure at file-open time
    }
  }

  // Forms 1 & 2: wordexp handles plain absolute paths and ${VAR} / `~` expansion.
  wordexp_t exp;
  std::string out;
  if (wordexp(in.c_str(), &exp, WRDE_NOCMD) == 0)
  {
    if (exp.we_wordc > 0 && exp.we_wordv[0] != nullptr)
    {
      out = exp.we_wordv[0];
    }
    wordfree(&exp);
  }
  if (out.empty())
  {
    out = in;
  }
  return out;
}

}  // namespace experimental_behaviors
