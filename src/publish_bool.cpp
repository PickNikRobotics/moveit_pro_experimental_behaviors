// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/publish_bool.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kDescriptionPublishBool = R"(
                <p>
                    Publishes a <code>std_msgs/Bool</code> message to the named topic.
                </p>
                <p>
                    The publisher is created lazily on the first tick and re-created if the
                    topic name changes between ticks. SystemDefaultsQoS (reliable).
                </p>
            )";

constexpr auto kPortIDTopicName = "topic_name";
constexpr auto kPortIDValue = "value";
}  // namespace

namespace experimental_behaviors
{
PublishBool::PublishBool(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList PublishBool::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDTopicName, "bool_topic",
                                      "The name of the std_msgs::msg::Bool topic to publish on."),
           BT::InputPort<bool>(kPortIDValue, false, "The boolean value to publish.") };
}

BT::KeyValueVector PublishBool::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Messages" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionPublishBool } };
}

BT::NodeStatus PublishBool::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDTopicName), getInput<bool>(kPortIDValue));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     "Failed to get required value from input port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [topic_name, value] = ports.value();

  if (!publisher_ || topic_name != current_topic_name_)
  {
    publisher_ =
        shared_resources_->node->create_publisher<std_msgs::msg::Bool>(topic_name, rclcpp::SystemDefaultsQoS());
    current_topic_name_ = topic_name;
  }

  std_msgs::msg::Bool msg;
  msg.data = value;
  publisher_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
