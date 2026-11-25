// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "point_cloud_behaviors/load_latest_point_cloud_from_file.hpp"

#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_pro_common/utils/filesystem_utils.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <optional>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
constexpr auto kPortDirectory = "directory";
constexpr auto kPortFilePrefix = "file_prefix";
constexpr auto kPortFrameID = "frame_id";
constexpr auto kPortPointCloud = "point_cloud";
constexpr auto kPortFilePath = "file_path";

// Same defaults as the file_path and file_prefix ports of SavePointCloudToFile.
constexpr auto kDefaultDirectory = "~/.config/moveit_pro/saved_behavior_data";
constexpr auto kDefaultFilePrefix = "pointcloud";
constexpr auto kDefaultFrameID = "world";
}  // namespace

namespace point_cloud_behaviors
{
namespace internal
{
tl::expected<std::filesystem::path, std::string> findNewestPcdFile(const std::filesystem::path& directory,
                                                                   const std::string& prefix)
{
  std::error_code ec;
  if (!std::filesystem::is_directory(directory, ec))
  {
    return tl::make_unexpected("'" + directory.string() + "' is not a directory");
  }

  std::optional<std::filesystem::path> newest;
  std::filesystem::file_time_type newest_time;
  for (const auto& entry : std::filesystem::directory_iterator(directory, ec))
  {
    if (!entry.is_regular_file(ec) || ec)
    {
      continue;
    }
    const auto& path = entry.path();
    if (path.extension() != ".pcd" || path.filename().string().rfind(prefix, 0) != 0)
    {
      continue;
    }
    const auto modified = std::filesystem::last_write_time(path, ec);
    if (ec)
    {
      continue;
    }
    if (!newest.has_value() || modified > newest_time)
    {
      newest = path;
      newest_time = modified;
    }
  }
  if (!newest.has_value())
  {
    return tl::make_unexpected("no '" + prefix + "*.pcd' file in '" + directory.string() + "'");
  }
  return newest.value();
}
}  // namespace internal

LoadLatestPointCloudFromFile::LoadLatestPointCloudFromFile(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList LoadLatestPointCloudFromFile::providedPorts()
{
  return { BT::InputPort<std::string>(kPortDirectory, kDefaultDirectory,
                                      "Directory to search. A leading '~' is your home directory."),
           BT::InputPort<std::string>(kPortFilePrefix, kDefaultFilePrefix,
                                      "File name prefix to match. The newest matching .pcd file is loaded. Empty "
                                      "matches every .pcd file."),
           BT::InputPort<std::string>(kPortFrameID, kDefaultFrameID, "Frame ID to set on the loaded point cloud."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>(kPortPointCloud, "{point_cloud}",
                                                         "The newest matching point cloud."),
           BT::OutputPort<std::string>(kPortFilePath, "{loaded_file_path}", "Full path of the file that was loaded.") };
}

BT::KeyValueVector LoadLatestPointCloudFromFile::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - 3D Point Cloud" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Loads the newest .pcd file in a directory whose name starts with a prefix, so you do not need the "
             "exact file name that SavePointCloudToFile made. The defaults match SavePointCloudToFile." } };
}

BT::NodeStatus LoadLatestPointCloudFromFile::tick()
{
  const auto directory = getInput<std::string>(kPortDirectory).value_or(kDefaultDirectory);
  const auto prefix = getInput<std::string>(kPortFilePrefix).value_or(kDefaultFilePrefix);
  const auto frame_id = getInput<std::string>(kPortFrameID).value_or(kDefaultFrameID);

  const auto expanded = moveit_pro::common::filesystem_utils::expandHomeFilePath(directory);
  if (!expanded.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(),
                                                        "Cannot expand '" + directory + "': " + expanded.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto newest = internal::findNewestPcdFile(expanded.value(), prefix);
  if (!newest.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Nothing to load: " + newest.error());
    return BT::NodeStatus::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  if (pcl::io::loadPCDFile(newest->string(), cloud) < 0)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot read PCD file '" + newest->string() + "'.");
    return BT::NodeStatus::FAILURE;
  }
  if (cloud.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "PCD file '" + newest->string() + "' has no points.");
    return BT::NodeStatus::FAILURE;
  }

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;

  setOutput(kPortPointCloud, message);
  setOutput(kPortFilePath, newest->string());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace point_cloud_behaviors
