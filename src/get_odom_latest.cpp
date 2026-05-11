// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_odom_latest.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kDescriptionGetOdomLatest = R"(
                <p>
                    Long-lived odometry subscriber that publishes the latest received message every tick.
                    Returns RUNNING forever; halted by the parent on parallel completion.
                </p>
                <p>
                    Unlike the core <code>GetOdom</code>, this behavior does not write to its output ports
                    until at least one real message has arrived. This avoids a cold-start race where
                    downstream consumers see a default-constructed Odometry (empty <code>header.frame_id</code>)
                    on early ticks and fail.
                </p>
            )";

constexpr auto kPortIdOdometryTopicName = "odometry_topic_name";
constexpr auto kPortIdOdometry = "subscribed_odometry";
constexpr auto kPortIdOdometryPose = "odometry_pose";
}  // namespace

namespace experimental_behaviors
{
GetOdomLatest::GetOdomLatest(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetOdomLatest::providedPorts()
{
  return BT::PortsList{ BT::InputPort<std::string>(kPortIdOdometryTopicName, "odometry_topic",
                                                   "The name of the nav_msgs::msg::Odometry topic to subscribe to."),
                        BT::OutputPort<nav_msgs::msg::Odometry>(kPortIdOdometry, "{subscribed_odometry}",
                                                                "Latest received odometry message."),
                        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIdOdometryPose, "{odometry_pose}",
                                                                        "Latest received odometry as a PoseStamped.") };
}

BT::KeyValueVector GetOdomLatest::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Navigation" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionGetOdomLatest } };
}

BT::NodeStatus GetOdomLatest::onStart()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIdOdometryTopicName));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Failed to get required value from input data port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }
  const auto& [odometry_topic_name] = ports.value();

  odometry_subscriber_ = shared_resources_->node->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_name, rclcpp::SystemDefaultsQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::unique_lock lock(odometry_mutex_);
        current_odometry_ = *msg;
      });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetOdomLatest::onRunning()
{
  std::shared_lock lock(odometry_mutex_);

  // Skip output until a real message has arrived. Without this, downstream consumers
  // see a default-constructed Odometry (empty header.frame_id) and fail.
  if (!current_odometry_.has_value())
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput(kPortIdOdometry, *current_odometry_);

  geometry_msgs::msg::PoseStamped odometry_pose;
  odometry_pose.header = current_odometry_->header;
  odometry_pose.pose = current_odometry_->pose.pose;
  setOutput(kPortIdOdometryPose, odometry_pose);

  return BT::NodeStatus::RUNNING;
}

void GetOdomLatest::onHalted()
{
  odometry_subscriber_.reset();
  std::unique_lock lock(odometry_mutex_);
  current_odometry_.reset();
}
}  // namespace experimental_behaviors
