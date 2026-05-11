// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_bool_instance.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIdBoolTopicName = "bool_topic_name";
constexpr auto kPortIdBoolValue = "subscribed_bool_instance";
}  // namespace

namespace experimental_behaviors
{
GetBoolInstance::GetBoolInstance(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetBoolInstance::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIdBoolTopicName, "bool_topic",
                                      "The name of the std_msgs::msg::Bool topic to subscribe to."),
           BT::OutputPort<std_msgs::msg::Bool>(kPortIdBoolValue, "{subscribed_bool_instance}",
                                               "Subscribed boolean message.") };
}

BT::KeyValueVector GetBoolInstance::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Messages" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Wait for one std_msgs::msg::Bool message on the given topic, then return SUCCESS. "
             "Snapshot/one-shot semantics: subscribes on start, blocks until first message, tears "
             "down its own subscription on success. Output is the full Bool message wrapper." } };
}

BT::NodeStatus GetBoolInstance::onStart()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIdBoolTopicName));
  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(name(), ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [bool_topic_name] = ports.value();

  if (!bool_subscriber_ || bool_topic_name != bool_topic_name_)
  {
    bool_topic_name_ = bool_topic_name;
    bool_subscriber_ = shared_resources_->node->create_subscription<std_msgs::msg::Bool>(
        bool_topic_name_, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::unique_lock lock(bool_mutex_);
          current_bool_ = *msg;
        });
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetBoolInstance::onRunning()
{
  std::optional<std_msgs::msg::Bool> bool_copy;
  {
    std::shared_lock lock(bool_mutex_);
    if (!current_bool_)
    {
      return BT::NodeStatus::RUNNING;
    }
    bool_copy = current_bool_;
  }

  setOutput(kPortIdBoolValue, *bool_copy);

  // Snapshot semantics: tear down subscription on success.
  bool_subscriber_.reset();

  return BT::NodeStatus::SUCCESS;
}

void GetBoolInstance::onHalted()
{
  bool_subscriber_.reset();
  std::unique_lock lock(bool_mutex_);
  current_bool_.reset();
}
}  // namespace experimental_behaviors
