// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/localization_behaviors/wait_for_message.hpp>

#include <fmt/format.h>

namespace experimental_behaviors::localization_utils
{
rclcpp::QoS matchPublisherQoS(const std::vector<rclcpp::TopicEndpointInfo>& publishers)
{
  bool any_best_effort = false;
  bool all_transient_local = !publishers.empty();
  for (const auto& publisher : publishers)
  {
    const auto& qos = publisher.qos_profile();
    any_best_effort |= qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort;
    all_transient_local &= qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  }
  rclcpp::QoS qos{ rclcpp::KeepLast(1) };
  qos.reliability(any_best_effort ? rclcpp::ReliabilityPolicy::BestEffort : rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(all_transient_local ? rclcpp::DurabilityPolicy::TransientLocal : rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

std::string waitForMessageError(WaitFailure failure, const std::string& topic, double timeout_seconds)
{
  switch (failure)
  {
    case WaitFailure::kInvalidTimeout:
      return fmt::format("Timeout for '{}' is {} s; it must be finite and not negative.", topic, timeout_seconds);
    case WaitFailure::kCancelledBeforePublisher:
      return fmt::format("Cancelled while waiting for a publisher on '{}'.", topic);
    case WaitFailure::kNoPublisher:
      return fmt::format("No publisher on '{}' within {} s.", topic, timeout_seconds);
    case WaitFailure::kCancelledBeforeMessage:
      return fmt::format("Cancelled while waiting for a message on '{}'.", topic);
    case WaitFailure::kNoMessage:
      return fmt::format("No message on '{}' within {} s.", topic, timeout_seconds);
  }
  return fmt::format("Failed to get a message on '{}'.", topic);
}
}  // namespace experimental_behaviors::localization_utils
