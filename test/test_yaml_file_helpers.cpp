// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/atomic_file_write.hpp>
#include <experimental_behaviors/path_expansion.hpp>

#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
using experimental_behaviors::AtomicWriteResult;
using experimental_behaviors::expandPath;
using experimental_behaviors::FileUpdateLock;
using experimental_behaviors::writeFileAtomically;

std::string readAll(const std::filesystem::path& path)
{
  std::ifstream in(path);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

class PathExpansion : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ::setenv("EXPERIMENTAL_BEHAVIORS_TEST_HOME_BACKUP", ::getenv("HOME") ? ::getenv("HOME") : "", 1);
    ::setenv("HOME", "/home/tester", 1);
    ::setenv("EXPERIMENTAL_BEHAVIORS_TEST_BASE", "/mnt/nvme/autowash", 1);
    ::unsetenv("EXPERIMENTAL_BEHAVIORS_TEST_UNSET");
  }

  void TearDown() override
  {
    ::setenv("HOME", ::getenv("EXPERIMENTAL_BEHAVIORS_TEST_HOME_BACKUP"), 1);
    ::unsetenv("EXPERIMENTAL_BEHAVIORS_TEST_BASE");
  }
};

TEST_F(PathExpansion, LeavesPlainAbsolutePathsAlone)
{
  EXPECT_EQ(expandPath("/mnt/nvme/autowash/job-7/manifest.yaml"), "/mnt/nvme/autowash/job-7/manifest.yaml");
}

TEST_F(PathExpansion, ExpandsBracedVariablesAndLeadingTilde)
{
  EXPECT_EQ(expandPath("${EXPERIMENTAL_BEHAVIORS_TEST_BASE}/job-7/manifest.yaml"),
            "/mnt/nvme/autowash/job-7/manifest.yaml");
  EXPECT_EQ(expandPath("~/manifest.yaml"), "/home/tester/manifest.yaml");
  EXPECT_EQ(expandPath("~"), "/home/tester");
}

TEST_F(PathExpansion, TreatsTheInputAsOnePathNotAShellWord)
{
  // A path is not field-split, so a directory name with a space survives whole.
  EXPECT_EQ(expandPath("/mnt/my job/manifest.yaml"), "/mnt/my job/manifest.yaml");
  // Nor is it glob-expanded: these must stay literal rather than resolving to
  // whichever file happens to match first.
  EXPECT_EQ(expandPath("/mnt/job-*/manifest.yaml"), "/mnt/job-*/manifest.yaml");
  EXPECT_EQ(expandPath("/mnt/job-?/manifest.yaml"), "/mnt/job-?/manifest.yaml");
  EXPECT_EQ(expandPath("/mnt/job[12]/manifest.yaml"), "/mnt/job[12]/manifest.yaml");
  // And no subshell runs.
  EXPECT_EQ(expandPath("/mnt/$(id -u)/manifest.yaml"), "/mnt/$(id -u)/manifest.yaml");
  EXPECT_EQ(expandPath("/mnt/`id -u`/manifest.yaml"), "/mnt/`id -u`/manifest.yaml");
  // A '$' that does not open "${" is an ordinary filename character.
  EXPECT_EQ(expandPath("/mnt/$HOME/manifest.yaml"), "/mnt/$HOME/manifest.yaml");
  // A '~' anywhere but the front is an ordinary filename character too.
  EXPECT_EQ(expandPath("/mnt/back~up/manifest.yaml"), "/mnt/back~up/manifest.yaml");
}

TEST_F(PathExpansion, FallsBackToTheLiteralInputWhenExpansionCannotSucceed)
{
  // An unset variable must not expand to "", which would silently retarget the
  // write; returning the literal makes the file-open error name what was asked for.
  EXPECT_EQ(expandPath("${EXPERIMENTAL_BEHAVIORS_TEST_UNSET}/manifest.yaml"),
            "${EXPERIMENTAL_BEHAVIORS_TEST_UNSET}/manifest.yaml");
  EXPECT_EQ(expandPath("${UNTERMINATED/manifest.yaml"), "${UNTERMINATED/manifest.yaml");
  EXPECT_EQ(expandPath("package://no_such_package_exists/manifest.yaml"),
            "package://no_such_package_exists/manifest.yaml");
}

TEST_F(PathExpansion, ResolvesPackageUrls)
{
  std::string share;
  try
  {
    share = ament_index_cpp::get_package_share_directory("experimental_behaviors");
  }
  catch (const std::exception& e)
  {
    GTEST_SKIP() << "package share directory not indexed in this environment: " << e.what();
  }
  EXPECT_EQ(expandPath("package://experimental_behaviors/config/manifest.yaml"), share + "/config/manifest.yaml");
}

class AtomicFileWrite : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() / "experimental_behaviors_atomic_write_test";
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    target_ = dir_ / "manifest.yaml";
  }

  void TearDown() override
  {
    std::filesystem::remove_all(dir_);
  }

  std::filesystem::path dir_;
  std::filesystem::path target_;
};

TEST_F(AtomicFileWrite, WritesAndThenReplacesTheDestination)
{
  std::string error;
  ASSERT_EQ(writeFileAtomically(target_, "first: 1\n", error), AtomicWriteResult::kSucceeded) << error;
  EXPECT_EQ(readAll(target_), "first: 1\n");

  ASSERT_EQ(writeFileAtomically(target_, "second: 2\n", error), AtomicWriteResult::kSucceeded) << error;
  EXPECT_EQ(readAll(target_), "second: 2\n");
}

TEST_F(AtomicFileWrite, LeavesNoTemporaryBehind)
{
  std::string error;
  ASSERT_EQ(writeFileAtomically(target_, "first: 1\n", error), AtomicWriteResult::kSucceeded) << error;

  std::vector<std::string> entries;
  for (const auto& entry : std::filesystem::directory_iterator(dir_))
  {
    entries.push_back(entry.path().filename().string());
  }
  EXPECT_EQ(entries, std::vector<std::string>{ "manifest.yaml" });
}

TEST_F(AtomicFileWrite, PreservesTheDestinationsPermissions)
{
  std::string error;
  ASSERT_EQ(writeFileAtomically(target_, "first: 1\n", error), AtomicWriteResult::kSucceeded) << error;

  const auto mode =
      std::filesystem::perms::owner_read | std::filesystem::perms::owner_write | std::filesystem::perms::group_read;
  std::filesystem::permissions(target_, mode);

  ASSERT_EQ(writeFileAtomically(target_, "second: 2\n", error), AtomicWriteResult::kSucceeded) << error;
  EXPECT_EQ(std::filesystem::status(target_).permissions() & std::filesystem::perms::mask, mode);
}

TEST_F(AtomicFileWrite, ReplacesTheTargetOfASymlinkedDestinationAndKeepsTheLink)
{
  const auto real = dir_ / "real.yaml";
  const auto link = dir_ / "link.yaml";
  std::string error;
  ASSERT_EQ(writeFileAtomically(real, "first: 1\n", error), AtomicWriteResult::kSucceeded) << error;
  std::filesystem::create_symlink(real, link);

  ASSERT_EQ(writeFileAtomically(link, "second: 2\n", error), AtomicWriteResult::kSucceeded) << error;

  // The link is still a link, and it is the file it names that changed.
  EXPECT_TRUE(std::filesystem::is_symlink(link));
  EXPECT_EQ(readAll(real), "second: 2\n");
}

TEST_F(AtomicFileWrite, DoesNotWriteThroughAPlantedTemporary)
{
  std::string error;
  ASSERT_EQ(writeFileAtomically(target_, "first: 1\n", error), AtomicWriteResult::kSucceeded) << error;

  // The temporary's name is predictable, so pre-create every name the next call
  // could pick as a symlink pointing somewhere it must not write. O_EXCL and
  // O_NOFOLLOW have to make the call either skip them or fail — never follow one.
  const auto victim = dir_ / "victim.yaml";
  ASSERT_EQ(writeFileAtomically(victim, "untouched\n", error), AtomicWriteResult::kSucceeded) << error;
  for (int i = 0; i < 16; ++i)
  {
    std::filesystem::create_symlink(
        victim, dir_ / ("manifest.yaml.tmp." + std::to_string(::getpid()) + "." + std::to_string(i)));
  }

  // Whether this call finds a free name or gives up, the victim must be intact.
  (void)writeFileAtomically(target_, "second: 2\n", error);
  EXPECT_EQ(readAll(victim), "untouched\n");
  // ...and the destination is either the old or the new document, never the
  // victim's content.
  const auto target_contents = readAll(target_);
  EXPECT_TRUE(target_contents == "first: 1\n" || target_contents == "second: 2\n") << target_contents;
}

TEST_F(AtomicFileWrite, ReportsFailureAndKeepsTheDestinationWhenTheWriteCannotLand)
{
  std::string error;
  ASSERT_EQ(writeFileAtomically(target_, "first: 1\n", error), AtomicWriteResult::kSucceeded) << error;

  // A destination whose parent directory does not exist cannot be written.
  error.clear();
  EXPECT_EQ(writeFileAtomically(dir_ / "absent_subdir" / "manifest.yaml", "x", error), AtomicWriteResult::kFailed);
  EXPECT_FALSE(error.empty());

  // The pre-existing file is untouched.
  EXPECT_EQ(readAll(target_), "first: 1\n");
}
class FileUpdateLocking : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() / "experimental_behaviors_file_lock_test";
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    target_ = dir_ / "manifest.yaml";
    std::string error;
    ASSERT_EQ(writeFileAtomically(target_, "items: []\n", error), AtomicWriteResult::kSucceeded) << error;
  }

  void TearDown() override
  {
    std::filesystem::remove_all(dir_);
  }

  std::filesystem::path dir_;
  std::filesystem::path target_;
};

TEST_F(FileUpdateLocking, LocksASidecarNextToTheTarget)
{
  std::string error;
  const auto lock = FileUpdateLock::acquire(target_, error);
  ASSERT_TRUE(lock.has_value()) << error;
  EXPECT_EQ(lock->lockPath(), dir_ / "manifest.yaml.lock");
  EXPECT_TRUE(std::filesystem::exists(lock->lockPath()));
}

TEST_F(FileUpdateLocking, DoesNotLockTheTargetItselfSoAnAtomicWriteStillWorksUnderIt)
{
  std::string error;
  const auto lock = FileUpdateLock::acquire(target_, error);
  ASSERT_TRUE(lock.has_value()) << error;

  // The lock guards the cycle; it must not block the replacement that ends it.
  ASSERT_EQ(writeFileAtomically(target_, "items: [a]\n", error), AtomicWriteResult::kSucceeded) << error;
  EXPECT_EQ(readAll(target_), "items: [a]\n");
}

TEST_F(FileUpdateLocking, ReachesTheSameLockThroughASymlinkedName)
{
  const auto link = dir_ / "link.yaml";
  std::filesystem::create_symlink(target_, link);

  std::filesystem::path direct_path;
  {
    std::string error;
    const auto direct = FileUpdateLock::acquire(target_, error);
    ASSERT_TRUE(direct.has_value()) << error;
    direct_path = direct->lockPath();
  }

  std::string error;
  const auto through_link = FileUpdateLock::acquire(link, error);
  ASSERT_TRUE(through_link.has_value()) << error;
  // Both names must resolve to one sidecar, or reaching a file through a link
  // would quietly bypass the exclusion.
  EXPECT_EQ(through_link->lockPath(), direct_path);
}

TEST_F(FileUpdateLocking, ExcludesASecondHolderAndReportsInsteadOfHanging)
{
  std::string error;
  const auto held = FileUpdateLock::acquire(target_, error);
  ASSERT_TRUE(held.has_value()) << error;

  // flock(2) is per-open-file-description, not per-process, so a second
  // acquisition contends with the first even from the same process — which is
  // exactly the case that matters: two behavior ticks in one objective server.
  error.clear();
  const auto start = std::chrono::steady_clock::now();
  const auto contended = FileUpdateLock::acquire(target_, error);
  const auto waited = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(contended.has_value());
  EXPECT_FALSE(error.empty());
  // It waited its bounded wait and then reported, rather than blocking forever.
  EXPECT_GE(waited, FileUpdateLock::kMaxWait);
  EXPECT_LT(waited, FileUpdateLock::kMaxWait + std::chrono::seconds(5));
}

TEST_F(FileUpdateLocking, LetsTheNextWriterInOnceTheHolderIsGone)
{
  {
    std::string error;
    const auto held = FileUpdateLock::acquire(target_, error);
    ASSERT_TRUE(held.has_value()) << error;
  }

  std::string error;
  const auto after = FileUpdateLock::acquire(target_, error);
  EXPECT_TRUE(after.has_value()) << error;
}

TEST_F(FileUpdateLocking, SerializesConcurrentReadModifyWriteCycles)
{
  // What the lock exists for: N threads each load, append one item, and replace.
  // Without it the last rename of each overlapping pair discards the other's
  // item and the file ends up with fewer than N.
  constexpr int kWriters = 8;
  std::vector<std::thread> writers;
  writers.reserve(kWriters);
  for (int i = 0; i < kWriters; ++i)
  {
    writers.emplace_back([this, i]() {
      std::string error;
      const auto lock = FileUpdateLock::acquire(target_, error);
      ASSERT_TRUE(lock.has_value()) << error;
      // Load, edit, replace — the same shape as the behaviors' tick(), with the
      // YAML round-trip stood in for by a line-append so the test needs no
      // yaml-cpp.
      std::string document = readAll(target_);
      document += "item-" + std::to_string(i) + "\n";
      ASSERT_EQ(writeFileAtomically(target_, document, error), AtomicWriteResult::kSucceeded) << error;
    });
  }
  for (auto& writer : writers)
  {
    writer.join();
  }

  const std::string final_document = readAll(target_);
  for (int i = 0; i < kWriters; ++i)
  {
    EXPECT_NE(final_document.find("item-" + std::to_string(i) + "\n"), std::string::npos)
        << "writer " << i << " was lost:\n"
        << final_document;
  }
}

TEST_F(FileUpdateLocking, LeavesTheSidecarBehind)
{
  std::string error;
  {
    const auto lock = FileUpdateLock::acquire(target_, error);
    ASSERT_TRUE(lock.has_value()) << error;
  }
  // The sidecar outlives the lock by design: unlinking it would let a second
  // writer create and lock a fresh file while the first still held the old one.
  EXPECT_TRUE(std::filesystem::exists(dir_ / "manifest.yaml.lock"));
}

TEST_F(FileUpdateLocking, RefusesToLockThroughAPlantedSidecarSymlink)
{
  const auto elsewhere = dir_ / "elsewhere.lock";
  std::filesystem::create_symlink(elsewhere, dir_ / "manifest.yaml.lock");

  std::string error;
  const auto lock = FileUpdateLock::acquire(target_, error);
  EXPECT_FALSE(lock.has_value());
  EXPECT_FALSE(error.empty());
  // A redirected lock would silently stop excluding anything, so nothing is
  // created at the symlink's target either.
  EXPECT_FALSE(std::filesystem::exists(elsewhere));
}

TEST_F(FileUpdateLocking, ReportsAnUnopenableSidecar)
{
  std::string error;
  const auto lock = FileUpdateLock::acquire(dir_ / "absent_subdir" / "manifest.yaml", error);
  EXPECT_FALSE(lock.has_value());
  EXPECT_FALSE(error.empty());
}
}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
