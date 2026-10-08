// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/localization_behaviors/get_pose_with_covariance_from_topic.hpp>

#include <fmt/format.h>
#include <experimental_behaviors/localization_behaviors/wait_for_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <cmath>

namespace
{
constexpr auto kPortTopicName = "topic_name";
constexpr auto kPortTimeout = "message_timeout_sec";
constexpr auto kPortPose = "pose_stamped";
constexpr auto kPortXStdDev = "x_stddev";
constexpr auto kPortYStdDev = "y_stddev";
constexpr auto kPortYawStdDev = "yaw_stddev";
constexpr auto kDefaultTimeoutSeconds = 5.0;

// Row-major 6x6 indices of the x, y and yaw variances.
constexpr std::size_t kIdxXX = 0;
constexpr std::size_t kIdxYY = 7;
constexpr std::size_t kIdxYawYaw = 35;

constexpr auto kDescription = R"(
    <p>Takes the next <code>geometry_msgs/PoseWithCovarianceStamped</code> from a topic, and outputs the pose and
    the standard deviations of its x, y and yaw terms.</p>
    <p>A latched (transient local) publisher is read at once. FAILURE if no message arrives before
    <code>message_timeout_sec</code>, or a variance is negative or not finite.</p>
)";
}  // namespace

namespace experimental_behaviors
{
namespace localization_utils
{
tl::expected<PlanarStdDev, std::string>
planarStdDev(const geometry_msgs::msg::PoseWithCovariance::_covariance_type& covariance)
{
  const auto stddev = [&](std::size_t index, const char* term) -> tl::expected<double, std::string> {
    const double variance = covariance[index];
    if (!std::isfinite(variance) || variance < 0.0)
    {
      return tl::make_unexpected(fmt::format("{} variance is {}, not a finite non-negative value.", term, variance));
    }
    return std::sqrt(variance);
  };
  const auto x = stddev(kIdxXX, "x");
  const auto y = stddev(kIdxYY, "y");
  const auto yaw = stddev(kIdxYawYaw, "yaw");
  for (const auto* term : { &x, &y, &yaw })
  {
    if (!term->has_value())
    {
      return tl::make_unexpected(term->error());
    }
  }
  return PlanarStdDev{ x.value(), y.value(), yaw.value() };
}
}  // namespace localization_utils

GetPoseWithCovarianceFromTopic::GetPoseWithCovarianceFromTopic(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources), context_{ shared_resources }
{
}

BT::PortsList GetPoseWithCovarianceFromTopic::providedPorts()
{
  return { BT::InputPort<std::string>(kPortTopicName, "geometry_msgs/PoseWithCovarianceStamped topic to read."),
           BT::InputPort<double>(kPortTimeout, kDefaultTimeoutSeconds, "Seconds to wait for a message."),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "{pose_stamped}", "The pose, without covariance."),
           BT::OutputPort<double>(kPortXStdDev, "{x_stddev}", "Standard deviation of x (m)."),
           BT::OutputPort<double>(kPortYStdDev, "{y_stddev}", "Standard deviation of y (m)."),
           BT::OutputPort<double>(kPortYawStdDev, "{yaw_stddev}", "Standard deviation of yaw (rad).") };
}

BT::KeyValueVector GetPoseWithCovarianceFromTopic::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Navigation" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

tl::expected<bool, std::string> GetPoseWithCovarianceFromTopic::doWork()
{
  cancel_ = false;
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortTopicName), getInput<double>(kPortTimeout));
  if (!ports)
  {
    return tl::make_unexpected("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [topic_name, timeout] = ports.value();

  notifyCanHalt();
  const auto message = localization_utils::waitForMessage<geometry_msgs::msg::PoseWithCovarianceStamped>(
      context_->node, context_->reentrant_callback_group, topic_name, std::chrono::duration<double>{ timeout }, cancel_);
  if (!message)
  {
    return tl::make_unexpected(message.error());
  }

  const auto stddev = localization_utils::planarStdDev(message->pose.covariance);
  if (!stddev)
  {
    return tl::make_unexpected("Message on '" + topic_name + "' has an invalid covariance: " + stddev.error());
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = message->header;
  pose.pose = message->pose.pose;
  setOutput(kPortPose, pose);
  setOutput(kPortXStdDev, stddev->x);
  setOutput(kPortYStdDev, stddev->y);
  setOutput(kPortYawStdDev, stddev->yaw);
  return true;
}

tl::expected<void, std::string> GetPoseWithCovarianceFromTopic::doHalt()
{
  cancel_ = true;
  return {};
}
}  // namespace experimental_behaviors
