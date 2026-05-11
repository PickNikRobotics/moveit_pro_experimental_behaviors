// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_empty_instance.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace experimental_behaviors
{
GetEmptyInstance::GetEmptyInstance(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
  , message_count_(0)
{
}

BT::PortsList GetEmptyInstance::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<std::string>("empty_topic_name", "empty_topic",
                                   "The name of the std_msgs::msg::Empty topic to subscribe to."),
        BT::OutputPort<bool>("message_received", "{message_received}",
                             "Boolean indicating if at least one message has been received."),
        BT::OutputPort<uint64_t>("message_count", "{message_count}", "Total count of empty messages received.") });
}

BT::KeyValueVector GetEmptyInstance::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Messages" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Wait for one std_msgs::msg::Empty message on the given topic, then return SUCCESS. "
             "Snapshot/one-shot semantics: subscribes on start, blocks until first message, tears "
             "down its own subscription on success." } };
}

BT::NodeStatus GetEmptyInstance::onStart()
{
  const auto topic_name_result = getInput<std::string>("empty_topic_name");
  if (!topic_name_result)
  {
    shared_resources_->logger->publishFailureMessage(
        name(),
        fmt::format("Failed to get required value from input port 'empty_topic_name': {}", topic_name_result.error()));
    return BT::NodeStatus::FAILURE;
  }

  const std::string empty_topic_name = topic_name_result.value();

  message_count_ = 0;

  empty_subscriber_ = shared_resources_->node->create_subscription<std_msgs::msg::Empty>(
      empty_topic_name, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        std::unique_lock lock(empty_mutex_);
        message_count_++;
      });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetEmptyInstance::onRunning()
{
  uint64_t count;
  {
    std::shared_lock lock(empty_mutex_);
    count = message_count_;
  }

  setOutput("message_received", count > 0);
  setOutput("message_count", count);

  if (count > 0)
  {
    empty_subscriber_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void GetEmptyInstance::onHalted()
{
  empty_subscriber_.reset();
  spdlog::info("GetEmptyInstance behavior halted.");
}
}  // namespace experimental_behaviors
