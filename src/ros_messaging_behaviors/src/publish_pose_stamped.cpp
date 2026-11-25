// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "ros_messaging_behaviors/publish_pose_stamped.hpp"

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace
{
constexpr auto kPortPose = "pose";
constexpr auto kPortTopicName = "topic_name";

inline constexpr auto kDescription = R"(
                <p>Publishes a <code>geometry_msgs/PoseStamped</code> from the blackboard to a latched topic, so other tools can show a pose the Objective computed, for example as a camera overlay.</p>
                <p>The publisher is transient-local and keeps the last pose, so a subscriber that starts after the tick still receives it. Exits with FAILURE when either port is unset, having published nothing.</p>
            )";
}  // namespace

namespace ros_messaging_behaviors
{
PublishPoseStamped::PublishPoseStamped(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList PublishPoseStamped::providedPorts()
{
  // No defaults: a default topic would merge two streams, and a default pose would publish a pose
  // the Objective never computed.
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "Pose to publish. Its header is kept as it is."),
    BT::InputPort<std::string>(kPortTopicName, "Topic to publish on. The topic is latched (transient-local)."),
  };
}

BT::KeyValueVector PublishPoseStamped::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "ROS Messaging" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus PublishPoseStamped::tick()
{
  const auto pose = getInput<geometry_msgs::msg::PoseStamped>(kPortPose);
  const auto topic = getInput<std::string>(kPortTopicName);
  if (!pose.has_value() || !topic.has_value() || topic.value().empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), std::string("Ports '") + kPortPose + "' and '" +
                                                                    kPortTopicName + "' are both required.");
    return BT::NodeStatus::FAILURE;
  }

  auto slot = publishers_.find(topic.value());
  if (slot == publishers_.end())
  {
    try
    {
      slot =
          publishers_
              .emplace(topic.value(), getBehaviorContext()->node->create_publisher<geometry_msgs::msg::PoseStamped>(
                                          topic.value(), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()))
              .first;
    }
    catch (const std::exception& e)
    {
      getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot create a publisher on topic '" +
                                                                      topic.value() + "': " + e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  slot->second->publish(pose.value());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros_messaging_behaviors
