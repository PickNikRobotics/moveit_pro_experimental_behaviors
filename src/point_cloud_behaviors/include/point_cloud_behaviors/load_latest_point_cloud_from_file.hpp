// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <tl_expected/expected.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace point_cloud_behaviors
{
namespace internal
{
/**
 * @brief The most recently modified regular file in @p directory whose name starts with @p prefix
 * and ends in ".pcd".
 * @return The path, or why there is none (no such directory, or no matching file).
 */
tl::expected<std::filesystem::path, std::string> findNewestPcdFile(const std::filesystem::path& directory,
                                                                   const std::string& prefix);
}  // namespace internal

/**
 * @brief Load the newest .pcd file in a directory whose name starts with a prefix.
 *
 * @details SavePointCloudToFile adds a timestamp to each file name, and LoadPointCloudFromFile needs
 * an exact path. This Behavior closes that gap; its defaults match SavePointCloudToFile.
 *
 * | Data Port Name | Port Type | Object Type                   |
 * | -------------- | --------- | ----------------------------- |
 * | directory      | Input     | std::string                   |
 * | file_prefix    | Input     | std::string                   |
 * | frame_id       | Input     | std::string                   |
 * | point_cloud    | Output    | sensor_msgs::msg::PointCloud2 |
 * | file_path      | Output    | std::string                   |
 */
class LoadLatestPointCloudFromFile final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  LoadLatestPointCloudFromFile(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};
}  // namespace point_cloud_behaviors
