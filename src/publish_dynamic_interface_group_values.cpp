// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/publish_dynamic_interface_group_values.hpp>
#include <moveit_studio_behavior_interface/metadata_fields.hpp>

namespace experimental_behaviors
{
inline constexpr auto kDescriptionPublishDynamicInterfaceGroupValues = R"(
                <p>
                    Publish a <code>control_msgs::msg::DynamicInterfaceGroupValues</code> message to a topic.
                </p>
            )";

PublishDynamicInterfaceGroupValues::PublishDynamicInterfaceGroupValues(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SendMessageToTopicBehaviorBase<control_msgs::msg::DynamicInterfaceGroupValues>(
        name, config, shared_resources)
{
}

BT::PortsList PublishDynamicInterfaceGroupValues::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<control_msgs::msg::DynamicInterfaceGroupValues>(
          kPortIDMessage, control_msgs::msg::DynamicInterfaceGroupValues(), "The message to be published"),
      BT::InputPort<std::string>(kPortIDTopicName, "", "The topic the message should be published to."),
      BT::InputPort<size_t>(kPortIDQueueSize, 1, "The queue size for the publisher."),
      BT::InputPort<bool>(kPortIDUseBestEffort, false,
                          "Whether the publisher's reliability should be best effort (true) or reliable (false)."),
  });
}

BT::KeyValueVector PublishDynamicInterfaceGroupValues::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "ROS Messaging" },
           { moveit_studio::behaviors::kDescriptionMetadataKey, kDescriptionPublishDynamicInterfaceGroupValues } };
}

PublishDynamicInterfaceGroupValues::PublishDynamicInterfaceGroupValues(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_studio::behaviors::PublisherInterfaceBase<control_msgs::msg::DynamicInterfaceGroupValues>>
        publisher_interface)
  : moveit_studio::behaviors::SendMessageToTopicBehaviorBase<control_msgs::msg::DynamicInterfaceGroupValues>(
        name, config, shared_resources, std::move(publisher_interface))
{
}

}  // namespace experimental_behaviors

template class moveit_studio::behaviors::SendMessageToTopicBehaviorBase<control_msgs::msg::DynamicInterfaceGroupValues>;
