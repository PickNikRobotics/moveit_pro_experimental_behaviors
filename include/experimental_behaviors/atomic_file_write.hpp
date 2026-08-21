// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

namespace experimental_behaviors
{

/**
 * @brief Replaces @p file_path with @p contents atomically.
 *
 * Writes to a uniquely named temporary file in the destination's own directory,
 * fsyncs it, then `rename(2)`s it over the destination and fsyncs the
 * directory. A concurrent reader therefore sees either the old file or the new
 * one, never a truncated one, and a crash cannot leave the destination
 * half-written. The temporary lives in the destination directory (not `/tmp`)
 * because `rename(2)` is only atomic within one filesystem.
 *
 * The destination's existing permissions are carried over to the replacement,
 * so an atomic write does not silently re-mode the file to the process umask.
 *
 * @param file_path Destination file. It need not exist yet.
 * @param contents Bytes to write.
 * @param[out] error Human-readable reason on failure. Untouched on success.
 * @return true if @p file_path now holds @p contents in full. On false the
 * destination is unchanged and any temporary has been removed.
 */
inline bool writeFileAtomically(const std::filesystem::path& file_path, std::string_view contents, std::string& error)
{
  // Unique per call: two behaviors ticking against the same destination must not
  // share a temporary. getpid() separates processes, the counter separates calls
  // within one process.
  static std::atomic<unsigned long long> counter{ 0 };
  const std::filesystem::path temp_path =
      file_path.parent_path() / (file_path.filename().string() + ".tmp." + std::to_string(::getpid()) + "." +
                                 std::to_string(counter.fetch_add(1)));

  const auto cleanup = [&temp_path]() {
    std::error_code ec;
    std::filesystem::remove(temp_path, ec);
  };

  {
    std::ofstream out(temp_path, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!out)
    {
      error = "failed to open temporary file '" + temp_path.string() + "' for writing";
      return false;
    }
    out.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    out.close();  // sets failbit if the flush-on-close fails, e.g. ENOSPC
    if (!out)
    {
      error = "failed to write temporary file '" + temp_path.string() + "'";
      cleanup();
      return false;
    }
  }

  // fsync the data before the rename: without it a crash can land the rename
  // while the bytes are still only in the page cache, leaving an empty file.
  {
    const int fd = ::open(temp_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
    {
      error = "failed to reopen temporary file '" + temp_path.string() + "' to flush it";
      cleanup();
      return false;
    }
    const int sync_result = ::fsync(fd);
    ::close(fd);
    if (sync_result != 0)
    {
      error = "failed to flush temporary file '" + temp_path.string() + "'";
      cleanup();
      return false;
    }
  }

  // Carry the destination's mode over, when there is a destination.
  std::error_code ec;
  const auto status = std::filesystem::status(file_path, ec);
  if (!ec && std::filesystem::is_regular_file(status))
  {
    std::filesystem::permissions(temp_path, status.permissions(), ec);
    if (ec)
    {
      error = "failed to apply '" + file_path.string() + "' permissions to the replacement: " + ec.message();
      cleanup();
      return false;
    }
  }

  std::filesystem::rename(temp_path, file_path, ec);
  if (ec)
  {
    error = "failed to move '" + temp_path.string() + "' onto '" + file_path.string() + "': " + ec.message();
    cleanup();
    return false;
  }

  // fsync the directory so the rename itself survives a crash.
  const std::filesystem::path dir = file_path.parent_path().empty() ? std::filesystem::path(".") : file_path.parent_path();
  const int dir_fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir_fd >= 0)
  {
    ::fsync(dir_fd);
    ::close(dir_fd);
  }

  return true;
}

}  // namespace experimental_behaviors
