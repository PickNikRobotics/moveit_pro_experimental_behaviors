// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>

#include <atomic>
#include <future>
#include <memory>
#include <string>

namespace experimental_behaviors
{
/**
 * @brief Scores a robot pose by how well the latest laser scan, placed at that pose, lands on the occupied cells
 * of a map, and fails when too few beams land close enough.
 *
 * | Data Port Name      | Port Type | Object Type                     |
 * | ------------------- | --------- | ------------------------------- |
 * | pose                | input     | geometry_msgs::msg::PoseStamped |
 * | robot_frame_id      | input     | std::string                     |
 * | scan_topic          | input     | std::string                     |
 * | map_topic           | input     | std::string                     |
 * | inlier_distance     | input     | double                          |
 * | min_inlier_fraction | input     | double                          |
 * | message_timeout_sec | input     | double                          |
 * | inlier_fraction     | output    | double                          |
 * | median_residual     | output    | double                          |
 * | beams_used          | output    | int                             |
 *
 * @details
 * Each usable beam's endpoint is placed in the map through the pose and the TF offset from robot_frame_id to the
 * scan's frame. Its residual is the distance to the nearest occupied cell; it is an inlier within inlier_distance.
 *
 * The scan and the map are read from their topics here. The outputs are set before the gate is applied, so they
 * can be logged when it fails. The pose must be in the map's frame.
 */
class ScanMatchResidualFromTopics final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  ScanMatchResidualFromTopics(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<bool, std::string> doWork() override;
  tl::expected<void, std::string> doHalt() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /// Own copy of the context: MoveIt Pro 9.x and 10.x name the base class's copy differently.
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  std::atomic<bool> cancel_{ false };
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace experimental_behaviors
