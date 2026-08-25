// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

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

/// Resolves @p file_path to the file that will actually be written: a symlink is
/// followed to the file it names, anything else is returned unchanged. Returns
/// std::nullopt and sets @p error when a link cannot be resolved.
inline std::optional<std::filesystem::path> resolveWriteDestination(const std::filesystem::path& file_path,
                                                                    std::string& error)
{
  std::error_code ec;
  if (!std::filesystem::is_symlink(file_path, ec))
  {
    return file_path;
  }
  const auto resolved = std::filesystem::weakly_canonical(file_path, ec);
  if (ec)
  {
    error = "failed to resolve the symlink at '" + file_path.string() + "': " + ec.message();
    return std::nullopt;
  }
  return resolved;
}

/// The directory @p destination lives in, as a path that can be opened.
inline std::filesystem::path containingDirectory(const std::filesystem::path& destination)
{
  return destination.parent_path().empty() ? std::filesystem::path(".") : destination.parent_path();
}
}  // namespace detail

/// Outcome of writeFileAtomically(). Three states, not two, because a failure
/// to make the rename durable is not a failure to write: the new contents are
/// already visible and re-running the write would repeat whatever edit produced
/// them.
enum class AtomicWriteResult
{
  /// @p file_path holds the new contents, and the replacement is durable.
  kSucceeded,
  /// @p file_path holds the new contents, but the directory entry may not
  /// survive a power loss. Not retriable: the write happened.
  kSucceededNotDurable,
  /// @p file_path is unchanged. Retriable.
  kFailed,
};

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
 * can still lose one of the two edits — the last rename wins. A caller doing a
 * load-edit-replace has to hold a FileUpdateLock (below) across the whole
 * sequence to get that guarantee.
 *
 * @param file_path Destination file. It need not exist yet.
 * @param contents Bytes to write.
 * @param[out] error Human-readable reason, set for both `kFailed` (why nothing
 * was written) and `kSucceededNotDurable` (what could not be flushed). Untouched
 * on `kSucceeded`.
 * @return Which of the three outcomes occurred. On `kFailed` the destination is
 * unchanged and any temporary has been removed, so the caller may retry. On
 * `kSucceededNotDurable` the write landed and must not be retried.
 */
inline AtomicWriteResult writeFileAtomically(const std::filesystem::path& file_path, std::string_view contents,
                                             std::string& error)
{
  // Follow a symlinked destination to the file it names: replacing the link
  // itself would silently change what every other reader of that path sees.
  const auto resolved = detail::resolveWriteDestination(file_path, error);
  if (!resolved)
  {
    return AtomicWriteResult::kFailed;
  }
  const std::filesystem::path destination = *resolved;
  std::error_code ec;

  const std::filesystem::path directory = detail::containingDirectory(destination);

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
      return AtomicWriteResult::kFailed;
    }
  }
  if (fd < 0)
  {
    error = "failed to find an unused temporary file name next to '" + destination.string() + "'";
    return AtomicWriteResult::kFailed;
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
    return AtomicWriteResult::kFailed;
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
    return AtomicWriteResult::kSucceededNotDurable;
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
    return AtomicWriteResult::kSucceededNotDurable;
  }

  return AtomicWriteResult::kSucceeded;
}

/**
 * @brief An exclusive lock held across one file's read-modify-write cycle.
 *
 * writeFileAtomically() makes the replacement indivisible; it cannot make a
 * load-edit-replace indivisible. Two writers can each load the same document,
 * apply a different edit, and the second rename discards the first edit. Holding
 * this lock from before the load until after the replacement serializes them, so
 * the second writer reads the first one's result and both edits survive.
 *
 * The lock is `flock(2)` on a sidecar file named `<destination>.lock`, not on the
 * destination itself: every atomic write replaces the destination's inode, so a
 * lock taken on it would be invisible to the next writer to open that path.
 * Being a real file lock, it serializes writers in separate processes as well as
 * concurrent ticks in one. It is released when the object is destroyed, and by
 * the kernel if the process dies holding it.
 *
 * The sidecar is created if absent and deliberately left behind — unlinking it
 * would let a second writer create and lock a fresh file while the first still
 * holds the old one. It stays empty.
 *
 * A symlinked destination is resolved the same way writeFileAtomically() resolves
 * it, so two names for one file take the same lock.
 */
class FileUpdateLock
{
public:
  /// How long acquire() waits for a competing holder. Every holder spans one
  /// load-edit-replace of a config-sized file, so still waiting after this long
  /// means something is wrong rather than merely busy — and failing the caller
  /// with a diagnosis beats stalling a behavior tree indefinitely.
  static constexpr std::chrono::seconds kMaxWait{ 5 };

  /**
   * @brief Takes the lock guarding writes to @p file_path.
   * @param file_path The file about to be read, edited and replaced.
   * @param[out] error Human-readable reason, set only when acquisition fails.
   * @return The held lock, or std::nullopt if it could not be taken.
   */
  static std::optional<FileUpdateLock> acquire(const std::filesystem::path& file_path, std::string& error)
  {
    const auto destination = detail::resolveWriteDestination(file_path, error);
    if (!destination)
    {
      return std::nullopt;
    }
    const std::filesystem::path lock_path =
        detail::containingDirectory(*destination) / (destination->filename().string() + ".lock");

    // O_RDONLY, not O_RDWR: flock(2) needs no write access, and a sidecar left
    // by another user may not grant it. O_NOFOLLOW so a planted symlink cannot
    // move the lock — and with it the mutual exclusion — somewhere else.
    const int fd = ::open(lock_path.c_str(), O_RDONLY | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0666);
    if (fd < 0)
    {
      error = "failed to open lock file '" + lock_path.string() + "': " + detail::errnoMessage(errno);
      return std::nullopt;
    }

    // Poll instead of blocking in flock(): a wait that cannot end has to become
    // an error the caller can report, not a hung tick.
    constexpr auto kPollInterval = std::chrono::milliseconds(5);
    const auto deadline = std::chrono::steady_clock::now() + kMaxWait;
    while (true)
    {
      if (::flock(fd, LOCK_EX | LOCK_NB) == 0)
      {
        return FileUpdateLock(fd, lock_path);
      }
      const int lock_errno = errno;
      if (lock_errno == EINTR)
      {
        continue;
      }
      if (lock_errno != EWOULDBLOCK)
      {
        error = "failed to lock '" + lock_path.string() + "': " + detail::errnoMessage(lock_errno);
        ::close(fd);
        return std::nullopt;
      }
      if (std::chrono::steady_clock::now() >= deadline)
      {
        error = "timed out after " + std::to_string(kMaxWait.count()) + "s waiting for another writer to release '" +
                lock_path.string() + "'";
        ::close(fd);
        return std::nullopt;
      }
      std::this_thread::sleep_for(kPollInterval);
    }
  }

  FileUpdateLock(const FileUpdateLock&) = delete;
  FileUpdateLock& operator=(const FileUpdateLock&) = delete;

  FileUpdateLock(FileUpdateLock&& other) noexcept : fd_{ other.fd_ }, lock_path_{ std::move(other.lock_path_) }
  {
    other.fd_ = -1;
  }

  FileUpdateLock& operator=(FileUpdateLock&& other) noexcept
  {
    if (this != &other)
    {
      release();
      fd_ = other.fd_;
      lock_path_ = std::move(other.lock_path_);
      other.fd_ = -1;
    }
    return *this;
  }

  ~FileUpdateLock()
  {
    release();
  }

  /// The sidecar file this lock is held on. Exposed for diagnostics and tests.
  const std::filesystem::path& lockPath() const
  {
    return lock_path_;
  }

private:
  FileUpdateLock(int fd, std::filesystem::path lock_path) : fd_{ fd }, lock_path_{ std::move(lock_path) }
  {
  }

  void release()
  {
    if (fd_ >= 0)
    {
      // close(2) drops the flock, so an explicit LOCK_UN would be redundant.
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_ = -1;
  std::filesystem::path lock_path_;
};

}  // namespace experimental_behaviors
