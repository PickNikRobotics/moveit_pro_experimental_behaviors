// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

namespace experimental_behaviors
{

namespace detail
{
/// Writes all of @p contents to @p fd, resuming through short writes and EINTR.
inline bool writeAll(int fd, std::string_view contents)
{
  const char* data = contents.data();
  std::size_t remaining = contents.size();
  while (remaining > 0)
  {
    const ssize_t written = ::write(fd, data, remaining);
    if (written < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      return false;
    }
    data += written;
    remaining -= static_cast<std::size_t>(written);
  }
  return true;
}

/// fsyncs @p fd, resuming through EINTR.
inline bool fsyncRetrying(int fd)
{
  while (::fsync(fd) != 0)
  {
    if (errno != EINTR)
    {
      return false;
    }
  }
  return true;
}

inline std::string errnoMessage(int error_number)
{
  return std::error_code(error_number, std::generic_category()).message();
}
}  // namespace detail

/**
 * @brief Replaces @p file_path with @p contents atomically.
 *
 * Creates a uniquely named temporary file in the destination's own directory
 * with `O_CREAT | O_EXCL | O_NOFOLLOW` (so it cannot be pre-empted by a planted
 * file or redirected through a symlink), writes @p contents through that
 * descriptor, fsyncs it, `rename(2)`s it over the destination and fsyncs the
 * directory. A concurrent reader therefore sees either the old file or the new
 * one, never a truncated one, and a crash cannot leave the destination
 * half-written. The temporary lives in the destination directory (not `/tmp`)
 * because `rename(2)` is only atomic within one filesystem.
 *
 * A destination that is a symlink is resolved first, so the link's target is
 * replaced and the link itself is left in place — matching what an ordinary
 * `open(2)` of the destination would have done.
 *
 * An existing destination's permissions are carried over to the replacement, so
 * an atomic write does not silently re-mode the file to the process umask. A
 * destination that does not exist yet is created with the same mode an
 * `std::ofstream` would have used (0666 less the umask).
 *
 * This serializes nothing: it replaces a file, it does not guard a
 * read-modify-write. Two writers that each load, edit and replace the same file
 * can still lose one of the two edits — the last rename wins. Callers that need
 * that guarantee have to serialize at a higher level.
 *
 * @param file_path Destination file. It need not exist yet.
 * @param contents Bytes to write.
 * @param[out] error Human-readable reason on failure. Untouched on success.
 * @return true if @p file_path now holds @p contents in full.
 *
 * On false the destination is unchanged and any temporary has been removed —
 * with one exception, called out in @p error: if the replacement itself landed
 * and only the final directory flush failed, the new contents are already
 * visible but the directory entry may not survive a power loss. That case
 * cannot be rolled back, so it is reported rather than hidden.
 */
inline bool writeFileAtomically(const std::filesystem::path& file_path, std::string_view contents, std::string& error)
{
  // Follow a symlinked destination to the file it names: replacing the link
  // itself would silently change what every other reader of that path sees.
  std::filesystem::path destination = file_path;
  std::error_code ec;
  if (std::filesystem::is_symlink(destination, ec))
  {
    const auto resolved = std::filesystem::weakly_canonical(destination, ec);
    if (ec)
    {
      error = "failed to resolve the symlink at '" + destination.string() + "': " + ec.message();
      return false;
    }
    destination = resolved;
  }

  const std::filesystem::path directory =
      destination.parent_path().empty() ? std::filesystem::path(".") : destination.parent_path();

  // The mode to give the replacement: the destination's own, or the umask
  // default when there is no destination yet.
  ::mode_t replacement_mode = 0;
  bool replacement_mode_known = false;
  struct ::stat destination_stat = {};
  if (::stat(destination.c_str(), &destination_stat) == 0 && S_ISREG(destination_stat.st_mode))
  {
    replacement_mode = destination_stat.st_mode & 07777;
    replacement_mode_known = true;
  }

  // O_EXCL means a planted file at the (predictable) temporary path makes the
  // create fail rather than being written through, so retry with a fresh name.
  // O_NOFOLLOW means a planted symlink cannot redirect the write either.
  static std::atomic<unsigned long long> counter{ 0 };
  constexpr int kMaxNameAttempts = 8;
  std::filesystem::path temp_path;
  int fd = -1;
  for (int attempt = 0; attempt < kMaxNameAttempts; ++attempt)
  {
    temp_path = directory / (destination.filename().string() + ".tmp." + std::to_string(::getpid()) + "." +
                             std::to_string(counter.fetch_add(1)));
    fd = ::open(temp_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0666);
    if (fd >= 0)
    {
      break;
    }
    if (errno != EEXIST)
    {
      error = "failed to create temporary file '" + temp_path.string() + "': " + detail::errnoMessage(errno);
      return false;
    }
  }
  if (fd < 0)
  {
    error = "failed to find an unused temporary file name next to '" + destination.string() + "'";
    return false;
  }

  const auto fail = [&](const std::string& reason) {
    error = reason;
    if (fd >= 0)
    {
      ::close(fd);
      fd = -1;
    }
    std::error_code remove_ec;
    std::filesystem::remove(temp_path, remove_ec);
    return false;
  };

  if (!detail::writeAll(fd, contents))
  {
    return fail("failed to write temporary file '" + temp_path.string() + "': " + detail::errnoMessage(errno));
  }

  // Carry the destination's mode over while the descriptor is still open, so no
  // path is re-resolved to do it.
  if (replacement_mode_known && ::fchmod(fd, replacement_mode) != 0)
  {
    return fail("failed to apply '" + destination.string() +
                "' permissions to the replacement: " + detail::errnoMessage(errno));
  }

  // fsync the data before the rename: without it a crash can land the rename
  // while the bytes are still only in the page cache, leaving an empty file.
  if (!detail::fsyncRetrying(fd))
  {
    return fail("failed to flush temporary file '" + temp_path.string() + "': " + detail::errnoMessage(errno));
  }
  if (::close(fd) != 0)
  {
    const std::string message =
        "failed to close temporary file '" + temp_path.string() + "': " + detail::errnoMessage(errno);
    fd = -1;
    return fail(message);
  }
  fd = -1;

  std::filesystem::rename(temp_path, destination, ec);
  if (ec)
  {
    return fail("failed to move '" + temp_path.string() + "' onto '" + destination.string() + "': " + ec.message());
  }

  // fsync the directory so the rename itself survives a power loss. Past this
  // point the new contents are already visible, so a failure here is reported
  // but cannot be undone.
  const int dir_fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir_fd < 0)
  {
    error = "wrote '" + destination.string() + "' but could not open '" + directory.string() +
            "' to flush the rename, so the update may not survive a power loss: " + detail::errnoMessage(errno);
    return false;
  }
  const bool synced = detail::fsyncRetrying(dir_fd);
  const int sync_errno = errno;
  ::close(dir_fd);
  // Filesystems that do not implement directory fsync report it rather than
  // failing to be durable, so those two errnos are not treated as failures.
  if (!synced && sync_errno != EINVAL && sync_errno != ENOTSUP)
  {
    error = "wrote '" + destination.string() + "' but could not flush '" + directory.string() +
            "', so the update may not survive a power loss: " + detail::errnoMessage(sync_errno);
    return false;
  }

  return true;
}

}  // namespace experimental_behaviors
