// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <tl/expected.hpp>

#include <atomic>
#include <future>
#include <memory>
#include <string>

namespace experimental_behaviors
{
namespace localization_utils
{
/** @brief Standard deviations of the planar pose terms of a covariance. */
struct PlanarStdDev
{
  double x;
  double y;
  double yaw;
};

/**
 * @brief Reads the x, y and yaw standard deviations off a row-major 6x6 pose covariance.
 * @return An error if any of those diagonal terms is negative or not finite.
 */
[[nodiscard]] tl::expected<PlanarStdDev, std::string>
planarStdDev(const geometry_msgs::msg::PoseWithCovariance::_covariance_type& covariance);
}  // namespace localization_utils

/**
 * @brief Takes the next geometry_msgs/PoseWithCovarianceStamped from a topic, and outputs the pose and its planar
 * standard deviations.
 *
 * | Data Port Name      | Port Type | Object Type                     |
 * | ------------------- | --------- | ------------------------------- |
 * | topic_name          | input     | std::string                     |
 * | message_timeout_sec | input     | double                          |
 * | pose_stamped        | output    | geometry_msgs::msg::PoseStamped |
 * | x_stddev            | output    | double                          |
 * | y_stddev            | output    | double                          |
 * | yaw_stddev          | output    | double                          |
 *
 * @details
 * A publisher that latches (transient local) is read at once; otherwise this waits for the next message.
 */
class GetPoseWithCovarianceFromTopic final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  GetPoseWithCovarianceFromTopic(const std::string& name, const BT::NodeConfiguration& config,
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
