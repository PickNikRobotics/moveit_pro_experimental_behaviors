// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_odom_instance.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIdOdomTopicName = "odom_topic_name";
constexpr auto kPortIdOdomValue = "subscribed_odom_instance";
}  // namespace

namespace experimental_behaviors
{
GetOdomInstance::GetOdomInstance(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetOdomInstance::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIdOdomTopicName, "odom",
                                      "The name of the nav_msgs::msg::Odometry topic to subscribe to."),
           BT::OutputPort<nav_msgs::msg::Odometry>(kPortIdOdomValue, "{subscribed_odom_instance}",
                                                   "Subscribed odometry message.") };
}

BT::KeyValueVector GetOdomInstance::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Messages" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Wait for one nav_msgs::msg::Odometry message on the given topic, then return SUCCESS. "
             "Snapshot/one-shot semantics: subscribes on start, blocks until first message, tears "
             "down its own subscription on success." } };
}

BT::NodeStatus GetOdomInstance::onStart()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIdOdomTopicName));
  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(name(), ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [odom_topic_name] = ports.value();

  // Recreate subscriber only if topic changed
  if (!odom_subscriber_ || odom_topic_name != odom_topic_name_)
  {
    odom_topic_name_ = odom_topic_name;
    odom_subscriber_ = shared_resources_->node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_, rclcpp::SystemDefaultsQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::unique_lock lock(odom_mutex_);
          current_odometry_ = *msg;
        });
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetOdomInstance::onRunning()
{
  std::optional<nav_msgs::msg::Odometry> odom_copy;
  {
    std::shared_lock lock(odom_mutex_);
    if (!current_odometry_)
    {
      return BT::NodeStatus::RUNNING;
    }
    odom_copy = current_odometry_;
  }

  setOutput(kPortIdOdomValue, *odom_copy);

  // Snapshot semantics: tear down subscription on success.
  odom_subscriber_.reset();

  return BT::NodeStatus::SUCCESS;
}

void GetOdomInstance::onHalted()
{
  odom_subscriber_.reset();
  std::unique_lock lock(odom_mutex_);
  current_odometry_.reset();
}
}  // namespace experimental_behaviors
