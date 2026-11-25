// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/publisher.hpp>

#include <map>
#include <memory>
#include <string>

namespace ros_messaging_behaviors
{
/**
 * @brief Publish a PoseStamped from the blackboard to a latched topic.
 *
 * @details The topic is transient-local, so a subscriber that starts after the tick still gets the
 * pose. Use it to show a computed pose in another tool, for example a camera overlay.
 *
 * Both ports are required. When one is missing, the tick fails and publishes nothing.
 *
 * | Data Port Name | Port Type | Object Type                     |
 * | -------------- | --------- | ------------------------------- |
 * | pose           | Input     | geometry_msgs::msg::PoseStamped |
 * | topic_name     | Input     | std::string                     |
 */
class PublishPoseStamped final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  PublishPoseStamped(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  // One latched publisher per topic, kept for the Behavior's life: a transient-local sample lives
  // in its publisher, so a publisher made only for one tick would drop the sample.
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;
};

}  // namespace ros_messaging_behaviors
