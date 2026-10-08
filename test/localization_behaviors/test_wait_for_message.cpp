// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/localization_behaviors/wait_for_message.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "localization_test_helpers.hpp"

#include <atomic>
#include <chrono>
#include <limits>

namespace
{
using experimental_behaviors::localization_utils::matchPublisherQoS;
using experimental_behaviors::localization_utils::waitForMessage;
using experimental_behaviors::test::SpunContext;
using Point = geometry_msgs::msg::Point;
using namespace std::chrono_literals;

// Publisher discovery is asynchronous, so poll the graph until the expected count shows up.
std::vector<rclcpp::TopicEndpointInfo> publishersOn(const rclcpp::Node& node, const std::string& topic,
                                                    std::size_t expected)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  auto publishers = node.get_publishers_info_by_topic(topic);
  while (publishers.size() < expected && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(10ms);
    publishers = node.get_publishers_info_by_topic(topic);
  }
  return publishers;
}

Point makeMessage(double value)
{
  Point message;
  message.x = value;
  return message;
}
}  // namespace

TEST(MatchPublisherQoS, NoPublisherGivesReliableVolatile)
{
  const auto qos = matchPublisherQoS({});
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(MatchPublisherQoS, AllLatchedPublishersGiveTransientLocal)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/qos_latched";
  const auto latched = rclcpp::QoS{ rclcpp::KeepLast(1) }.reliable().transient_local();
  const auto first = spun.node->create_publisher<Point>(topic, latched);
  const auto second = spun.node->create_publisher<Point>(topic, latched);
  const auto qos = matchPublisherQoS(publishersOn(*spun.node, topic, 2));
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(MatchPublisherQoS, OneBestEffortOrVolatilePublisherDowngrades)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/qos_mixed";
  const auto latched =
      spun.node->create_publisher<Point>(topic, rclcpp::QoS{ rclcpp::KeepLast(1) }.reliable().transient_local());
  const auto sensor = spun.node->create_publisher<Point>(topic, rclcpp::SensorDataQoS());
  const auto qos = matchPublisherQoS(publishersOn(*spun.node, topic, 2));
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(WaitForMessage, ReceivesFromABestEffortPublisher)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/best_effort";
  const auto publisher = spun.node->create_publisher<Point>(topic, rclcpp::SensorDataQoS());
  const auto timer = spun.node->create_wall_timer(50ms, [&publisher] { publisher->publish(makeMessage(7.0)); });
  std::atomic<bool> cancel{ false };
  const auto message = waitForMessage<Point>(spun.node, spun.context->reentrant_callback_group, topic,
                                             std::chrono::duration<double>{ 3.0 }, cancel);
  ASSERT_TRUE(message.has_value()) << message.error();
  EXPECT_DOUBLE_EQ(message->x, 7.0);
}

TEST(WaitForMessage, ReceivesALatchedMessagePublishedEarlier)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/latched";
  const auto publisher =
      spun.node->create_publisher<Point>(topic, rclcpp::QoS{ rclcpp::KeepLast(1) }.reliable().transient_local());
  publisher->publish(makeMessage(3.0));
  std::atomic<bool> cancel{ false };
  const auto message = waitForMessage<Point>(spun.node, spun.context->reentrant_callback_group, topic,
                                             std::chrono::duration<double>{ 3.0 }, cancel);
  ASSERT_TRUE(message.has_value()) << message.error();
  EXPECT_DOUBLE_EQ(message->x, 3.0);
}

TEST(WaitForMessage, TimesOutWithoutAPublisher)
{
  SpunContext spun;
  std::atomic<bool> cancel{ false };
  const auto message =
      waitForMessage<Point>(spun.node, spun.context->reentrant_callback_group, "/localization_behaviors_test/nobody",
                            std::chrono::duration<double>{ 0.2 }, cancel);
  ASSERT_FALSE(message.has_value());
  EXPECT_NE(message.error().find("No publisher"), std::string::npos) << message.error();
}

TEST(WaitForMessage, TimesOutWhenThePublisherIsSilent)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/silent";
  const auto publisher = spun.node->create_publisher<Point>(topic, rclcpp::QoS{ 10 });
  std::atomic<bool> cancel{ false };
  const auto message = waitForMessage<Point>(spun.node, spun.context->reentrant_callback_group, topic,
                                             std::chrono::duration<double>{ 0.3 }, cancel);
  ASSERT_FALSE(message.has_value());
  EXPECT_NE(message.error().find("No message"), std::string::npos) << message.error();
}

TEST(WaitForMessage, StopsWhenCancelled)
{
  SpunContext spun;
  std::atomic<bool> cancel{ true };
  const auto start = std::chrono::steady_clock::now();
  const auto message =
      waitForMessage<Point>(spun.node, spun.context->reentrant_callback_group, "/localization_behaviors_test/cancelled",
                            std::chrono::duration<double>{ 30.0 }, cancel);
  ASSERT_FALSE(message.has_value());
  EXPECT_NE(message.error().find("Cancelled"), std::string::npos) << message.error();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}

TEST(WaitForMessage, RejectsAnInvalidTimeout)
{
  SpunContext spun;
  std::atomic<bool> cancel{ false };
  for (const double timeout :
       { -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
  {
    const auto message =
        waitForMessage<Point>(spun.node, spun.context->reentrant_callback_group, "/localization_behaviors_test/any",
                              std::chrono::duration<double>{ timeout }, cancel);
    EXPECT_FALSE(message.has_value()) << timeout;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
