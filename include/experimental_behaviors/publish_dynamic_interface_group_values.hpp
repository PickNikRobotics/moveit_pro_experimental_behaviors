// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/basic_types.h>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>

#include <memory>
#include <moveit_studio_behavior_interface/send_message_to_topic.hpp>

namespace experimental_behaviors
{

/**
 * @brief Publish a control_msgs::msg::DynamicInterfaceGroupValues message to a topic.
 *
 * @details
 * | Data Port Name  | Port Type | Object Type          |
 * |-----------------|-----------|----------------------|
 * | message         | Input     | control_msgs::msg::DynamicInterfaceGroupValues |
 * | topic           | Input     | std::string          |
 * | queue_size      | Input     | size_t               |
 * | use_best_effort | Input     | bool                 |
 */
class PublishDynamicInterfaceGroupValues final : public moveit_studio::behaviors::SendMessageToTopicBehaviorBase<control_msgs::msg::DynamicInterfaceGroupValues>
{
public:
  PublishDynamicInterfaceGroupValues(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  PublishDynamicInterfaceGroupValues(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
               std::unique_ptr<moveit_studio::behaviors::PublisherInterfaceBase<control_msgs::msg::DynamicInterfaceGroupValues>> publisher_interface);
};

}  // namespace experimental_behaviors
