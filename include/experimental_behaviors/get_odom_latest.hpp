// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <optional>
#include <shared_mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/subscription.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Long-lived odometry subscriber that publishes the latest received message every tick.
 *
 * @details Like the core `GetOdom` behavior in shape — `StatefulActionNode`, returns RUNNING
 *          forever, halted by the parent on parallel completion — but does NOT write anything
 *          to its output ports until at least one real message has arrived. This avoids a
 *          cold-start race where downstream consumers (e.g. CalculatePoseOffset) see a
 *          default-constructed Odometry with an empty `header.frame_id` on early ticks and
 *          fail the tree.
 *
 *          Intended to be used as a producer in a Parallel, with the consumer reading
 *          `odometry_pose` from the blackboard. Pair with a seed (e.g. ConvertOdomToPoseStamped
 *          on a fresh GetOdomInstance snapshot) so the consumer has a valid value to read on
 *          its first tick before this behavior has received anything.
 *
 * | Data Port Name       | Port Type | Object Type                     |
 * | -------------------- | --------- | ------------------------------- |
 * | odometry_topic_name  | input     | std::string                     |
 * | subscribed_odometry  | output    | nav_msgs::msg::Odometry         |
 * | odometry_pose        | output    | geometry_msgs::msg::PoseStamped |
 */
class GetOdomLatest final : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetOdomLatest(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odometry_subscriber_;
  std::shared_mutex odometry_mutex_;
  std::optional<nav_msgs::msg::Odometry> current_odometry_;
};
}  // namespace experimental_behaviors
