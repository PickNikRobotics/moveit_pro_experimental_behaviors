// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "ros_messaging_behaviors/maintain_topic_subscription.hpp"

#include <sstream>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIDTopicNames = "topic_names";

inline constexpr auto kDescriptionMaintainTopicSubscription = R"(
                <p>Holds subscriptions to one or more topics, of any message type, open while this Behavior is RUNNING, and drops them when it is halted.</p>
                <p>Some drivers publish a stream only while it has a subscriber, for example a depth camera that computes depth only on demand. Run this Behavior in a Parallel next to the steps that read the stream, such as GetPointCloud or GetCameraInfo. When those steps finish, the Parallel halts this Behavior and the driver can stop the stream again.</p>
                <p>The message type of each topic is read from the ROS graph, so you do not set it. A topic that has no publisher yet is retried on every tick.</p>
            )";

// Split a semicolon-separated string into trimmed, non-empty topic names.
std::vector<std::string> splitTopics(const std::string& topics)
{
  std::vector<std::string> result;
  std::stringstream stream(topics);
  std::string item;
  while (std::getline(stream, item, ';'))
  {
    const auto begin = item.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
      continue;
    }
    const auto end = item.find_last_not_of(" \t\r\n");
    result.push_back(item.substr(begin, end - begin + 1));
  }
  return result;
}
}  // namespace

namespace ros_messaging_behaviors
{
MaintainTopicSubscription::MaintainTopicSubscription(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList MaintainTopicSubscription::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDTopicNames, "Semicolon-separated list of topics to keep a subscription "
                                                         "open on while this Behavior runs.") };
}

BT::KeyValueVector MaintainTopicSubscription::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "ROS Messaging" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionMaintainTopicSubscription } };
}

BT::NodeStatus MaintainTopicSubscription::onStart()
{
  const auto topic_names = getInput<std::string>(kPortIDTopicNames);
  if (!topic_names.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Missing required input port [" + std::string(kPortIDTopicNames) + "]: " + topic_names.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto topics = splitTopics(topic_names.value());
  if (topics.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "No topics found in [" + std::string(kPortIDTopicNames) + "]: '" + topic_names.value() + "'");
    return BT::NodeStatus::FAILURE;
  }

  // A topic with no publisher yet stays pending instead of failing: during start-up the publisher
  // can appear a moment later, and only then is its message type known.
  subscriptions_.clear();
  pending_topics_.clear();
  for (const auto& topic : topics)
  {
    if (!trySubscribe(topic))
    {
      pending_topics_.push_back(topic);
    }
  }

  RCLCPP_INFO(getBehaviorContext()->node->get_logger(),
              "MaintainTopicSubscription: holding %zu subscription(s) open, %zu waiting for a publisher.",
              subscriptions_.size(), pending_topics_.size());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MaintainTopicSubscription::onRunning()
{
  if (!pending_topics_.empty())
  {
    std::vector<std::string> still_pending;
    for (const auto& topic : pending_topics_)
    {
      if (!trySubscribe(topic))
      {
        still_pending.push_back(topic);
      }
    }
    pending_topics_ = std::move(still_pending);
  }
  return BT::NodeStatus::RUNNING;
}

void MaintainTopicSubscription::onHalted()
{
  subscriptions_.clear();
  pending_topics_.clear();
}

bool MaintainTopicSubscription::trySubscribe(const std::string& topic)
{
  const auto endpoints = getBehaviorContext()->node->get_publishers_info_by_topic(topic);
  if (endpoints.empty())
  {
    return false;
  }

  // Best effort matches both best-effort and reliable publishers, so the subscription always counts.
  // The callback does nothing: other Behaviors read the data.
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group = getBehaviorContext()->reentrant_callback_group;
  try
  {
    subscriptions_.push_back(getBehaviorContext()->node->create_generic_subscription(
        topic, endpoints.front().topic_type(), rclcpp::SensorDataQoS(),
        [](std::shared_ptr<rclcpp::SerializedMessage> /*message*/) {}, subscription_options));
  }
  catch (const std::exception& e)
  {
    // A retry cannot fix a type support or creation error, so report it and stop retrying this topic.
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to create subscription on topic '" + topic +
                                                                    "': " + e.what());
  }
  return true;
}

}  // namespace ros_messaging_behaviors
