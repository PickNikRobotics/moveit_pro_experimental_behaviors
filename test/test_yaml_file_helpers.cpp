// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/atomic_file_write.hpp>
#include <experimental_behaviors/path_expansion.hpp>

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using experimental_behaviors::AtomicWriteResult;
using experimental_behaviors::expandPath;
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
}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
