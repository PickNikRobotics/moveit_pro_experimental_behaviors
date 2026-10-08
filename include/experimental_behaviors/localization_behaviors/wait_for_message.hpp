// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tl/expected.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace experimental_behaviors::localization_utils
{
/**
 * @brief Picks a subscription QoS that every current publisher on a topic can serve.
 * @details Best effort if any publisher is best effort, and transient local only if all of them are, so a latched
 * message such as a map or a last pose estimate is delivered to a late subscriber.
 */
rclcpp::QoS matchPublisherQoS(const std::vector<rclcpp::TopicEndpointInfo>& publishers);

/** @brief Why waitForMessage gave up. */
enum class WaitFailure
{
  kInvalidTimeout,
  kCancelledBeforePublisher,
  kNoPublisher,
  kCancelledBeforeMessage,
  kNoMessage,
};

/** @brief Error text for waitForMessage, kept out of this header so it needs no formatting library. */
[[nodiscard]] std::string waitForMessageError(WaitFailure failure, const std::string& topic, double timeout_seconds);

/**
 * @brief Blocks until one message arrives on @p topic, @p timeout elapses, or @p cancel is set.
 * @details The subscription is made in @p group on @p node, which the caller's executor must spin.
 * @return The message, or an error naming the topic and the reason. A negative or non-finite timeout is an error.
 */
template <typename MessageT>
tl::expected<MessageT, std::string>
waitForMessage(const std::shared_ptr<rclcpp::Node>& node, const std::shared_ptr<rclcpp::CallbackGroup>& group,
               const std::string& topic, std::chrono::duration<double> timeout, const std::atomic<bool>& cancel)
{
  using namespace std::chrono_literals;
  if (!std::isfinite(timeout.count()) || timeout.count() < 0.0)
  {
    return tl::make_unexpected(waitForMessageError(WaitFailure::kInvalidTimeout, topic, timeout.count()));
  }
  // Elapsed time is compared in double seconds, so a huge timeout cannot overflow the clock's integer ticks.
  const auto start = std::chrono::steady_clock::now();
  const auto expired = [&] { return std::chrono::steady_clock::now() - start >= timeout; };

  // QoS is negotiated from the publishers, so wait for at least one to exist.
  auto publishers = node->get_publishers_info_by_topic(topic);
  while (publishers.empty())
  {
    if (cancel)
    {
      return tl::make_unexpected(waitForMessageError(WaitFailure::kCancelledBeforePublisher, topic, timeout.count()));
    }
    if (expired())
    {
      return tl::make_unexpected(waitForMessageError(WaitFailure::kNoPublisher, topic, timeout.count()));
    }
    std::this_thread::sleep_for(10ms);
    publishers = node->get_publishers_info_by_topic(topic);
  }

  auto promise = std::make_shared<std::promise<MessageT>>();
  auto future = promise->get_future();
  auto received = std::make_shared<std::atomic<bool>>(false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  const auto subscription = node->create_subscription<MessageT>(
      topic, matchPublisherQoS(publishers),
      [promise, received](const MessageT& message) {
        if (!received->exchange(true))
        {
          promise->set_value(message);
        }
      },
      options);

  while (future.wait_for(10ms) != std::future_status::ready)
  {
    if (cancel)
    {
      return tl::make_unexpected(waitForMessageError(WaitFailure::kCancelledBeforeMessage, topic, timeout.count()));
    }
    if (expired())
    {
      return tl::make_unexpected(waitForMessageError(WaitFailure::kNoMessage, topic, timeout.count()));
    }
  }
  return future.get();
}
}  // namespace experimental_behaviors::localization_utils
