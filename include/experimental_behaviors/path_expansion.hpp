// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <cstdlib>
#include <string>
#include <string_view>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace experimental_behaviors
{

namespace detail
{
/**
 * @brief Substitutes `${VAR}` references and a leading `~` in @p in, writing
 * the result to @p out.
 *
 * Every other character — spaces, quotes, `*`, `?`, `[` — is copied through
 * verbatim: a file path is a single path, not a shell word, so it must not be
 * field-split or glob-expanded.
 *
 * @return true on success. False if a `${VAR}` reference is malformed or names
 * an unset variable, or if a leading `~` cannot be resolved because `HOME` is
 * unset; @p out is untouched in that case, so the caller can keep the literal
 * input and let the file-open error name the path the tree actually asked for.
 */
inline bool expandVariables(const std::string& in, std::string& out)
{
  std::string result;
  result.reserve(in.size());

  std::size_t i = 0;
  if (in == "~" || in.rfind("~/", 0) == 0)
  {
    const char* home = std::getenv("HOME");
    if (home == nullptr || *home == '\0')
    {
      return false;
    }
    result += home;
    i = 1;  // the '/' (if any) is copied by the loop below
  }

  while (i < in.size())
  {
    // Only "${VAR}" expands. A bare '$' — including "$VAR" — is a literal
    // character in a filename, so it is copied through.
    if (in[i] == '$' && i + 1 < in.size() && in[i + 1] == '{')
    {
      const auto close = in.find('}', i + 2);
      if (close == std::string::npos)
      {
        return false;  // unterminated ${
      }
      const std::string name = in.substr(i + 2, close - (i + 2));
      const char* value = name.empty() ? nullptr : std::getenv(name.c_str());
      if (value == nullptr)
      {
        return false;  // unset variable — silently expanding to "" would corrupt the path
      }
      result += value;
      i = close + 1;
      continue;
    }
    result += in[i];
    ++i;
  }

  out = std::move(result);
  return true;
}
}  // namespace detail

/**
 * @brief Resolves a path string to an absolute filesystem path.
 *
 * Accepts three forms:
 *
 *   1. **Plain absolute path** — e.g. `/mnt/nvme/autowash/foo.yaml`.
 *      Returned unchanged.
 *   2. **Absolute path with variable expansion** — `${VAR}` and a
 *      leading `~` are expanded against the calling process's
 *      environment. Nothing else is interpreted: the input is treated
 *      as one path, so spaces and glob characters survive intact and no
 *      command substitution is possible.
 *   3. **ROS package URL** — `package://<package_name>/<rest>` resolves
 *      `<package_name>` via
 *      `ament_index_cpp::get_package_share_directory` and prepends the
 *      result to `<rest>` (mirrors the convention used by URDF
 *      `<mesh filename="package://...">`).
 *
 * Returns the resolved absolute path. On any expansion failure (unknown
 * package, unset or malformed variable, etc.) falls back to returning the
 * literal input; downstream `std::filesystem::exists` / `YAML::LoadFile`
 * calls surface the resulting failure to the caller with a useful error.
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

  // Forms 1 & 2.
  std::string out;
  if (!detail::expandVariables(in, out))
  {
    return in;  // literal fallback — caller surfaces the failure at file-open time
  }
  return out;
}

}  // namespace experimental_behaviors
