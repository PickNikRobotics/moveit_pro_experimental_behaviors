// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/localization_behaviors/get_pose_with_covariance_from_topic.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "localization_test_helpers.hpp"

#include <chrono>
#include <cmath>
#include <limits>

namespace
{
using experimental_behaviors::GetPoseWithCovarianceFromTopic;
using experimental_behaviors::test::SpunContext;
using experimental_behaviors::test::tickUntilDone;
using geometry_msgs::msg::PoseWithCovarianceStamped;

PoseWithCovarianceStamped makeMessage(double xx, double yy, double yawyaw)
{
  PoseWithCovarianceStamped message;
  message.header.frame_id = "map";
  message.pose.pose.position.x = 1.5;
  message.pose.pose.position.y = -2.0;
  message.pose.pose.orientation.w = 1.0;
  message.pose.covariance[0] = xx;
  message.pose.covariance[7] = yy;
  message.pose.covariance[35] = yawyaw;
  return message;
}

BT::NodeConfiguration makeConfig(const std::string& topic)
{
  auto config = experimental_behaviors::test::makeConfig();
  config.input_ports["topic_name"] = topic;
  config.input_ports["message_timeout_sec"] = "3.0";
  for (const auto* port : { "pose_stamped", "x_stddev", "y_stddev", "yaw_stddev" })
  {
    config.output_ports[port] = std::string{ "{" } + port + "}";
  }
  return config;
}

rclcpp::QoS latchedQoS()
{
  return rclcpp::QoS{ rclcpp::KeepLast(1) }.reliable().transient_local();
}
}  // namespace

TEST(PlanarStdDev, TakesTheSquareRootOfTheDiagonal)
{
  const auto stddev =
      experimental_behaviors::localization_utils::planarStdDev(makeMessage(0.04, 0.09, 0.01).pose.covariance);
  ASSERT_TRUE(stddev.has_value());
  EXPECT_DOUBLE_EQ(stddev->x, 0.2);
  EXPECT_DOUBLE_EQ(stddev->y, 0.3);
  EXPECT_DOUBLE_EQ(stddev->yaw, 0.1);
}

TEST(PlanarStdDev, RejectsNegativeOrNonFiniteVariance)
{
  using experimental_behaviors::localization_utils::planarStdDev;
  EXPECT_FALSE(planarStdDev(makeMessage(-0.1, 0.0, 0.0).pose.covariance).has_value());
  EXPECT_FALSE(
      planarStdDev(makeMessage(0.0, 0.0, std::numeric_limits<double>::quiet_NaN()).pose.covariance).has_value());
  EXPECT_FALSE(planarStdDev(makeMessage(0.0, std::numeric_limits<double>::infinity(), 0.0).pose.covariance).has_value());
}

TEST(PlanarStdDev, IgnoresOffDiagonalAndNonPlanarTerms)
{
  auto message = makeMessage(0.04, 0.09, 0.01);
  message.pose.covariance[1] = -5.0;   // x-y correlation
  message.pose.covariance[14] = -5.0;  // z variance
  const auto stddev = experimental_behaviors::localization_utils::planarStdDev(message.pose.covariance);
  ASSERT_TRUE(stddev.has_value());
  EXPECT_DOUBLE_EQ(stddev->x, 0.2);
}

TEST(GetPoseWithCovarianceFromTopic, ReadsALatchedMessage)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/latched_pose";
  // Published once before the behavior subscribes, as a localizer latches its last estimate.
  const auto publisher = spun.node->create_publisher<PoseWithCovarianceStamped>(topic, latchedQoS());
  publisher->publish(makeMessage(0.04, 0.09, 0.01));

  auto config = makeConfig(topic);
  GetPoseWithCovarianceFromTopic behavior{ "GetPoseWithCovarianceFromTopic", config, spun.context };
  ASSERT_EQ(tickUntilDone(behavior), BT::NodeStatus::SUCCESS);

  const auto pose = config.blackboard->get<geometry_msgs::msg::PoseStamped>("pose_stamped");
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("x_stddev"), 0.2);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("y_stddev"), 0.3);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("yaw_stddev"), 0.1);
}

TEST(GetPoseWithCovarianceFromTopic, ReadsTheNextMessageFromAVolatilePublisher)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/streamed_pose";
  const auto publisher = spun.node->create_publisher<PoseWithCovarianceStamped>(topic, rclcpp::QoS{ 10 });
  const auto timer = spun.node->create_wall_timer(std::chrono::milliseconds{ 50 },
                                                  [&publisher] { publisher->publish(makeMessage(0.04, 0.09, 0.01)); });

  auto config = makeConfig(topic);
  GetPoseWithCovarianceFromTopic behavior{ "GetPoseWithCovarianceFromTopic", config, spun.context };
  ASSERT_EQ(tickUntilDone(behavior), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("yaw_stddev"), 0.1);
}

TEST(GetPoseWithCovarianceFromTopic, FailsWhenNothingIsPublished)
{
  SpunContext spun;
  auto config = makeConfig("/localization_behaviors_test/silent_pose");
  config.input_ports["message_timeout_sec"] = "0.3";
  GetPoseWithCovarianceFromTopic behavior{ "GetPoseWithCovarianceFromTopic", config, spun.context };
  EXPECT_EQ(tickUntilDone(behavior), BT::NodeStatus::FAILURE);
}

TEST(GetPoseWithCovarianceFromTopic, FailsOnAnInvalidCovariance)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/bad_pose";
  const auto publisher = spun.node->create_publisher<PoseWithCovarianceStamped>(topic, latchedQoS());
  publisher->publish(makeMessage(-1.0, 0.0, 0.0));
  auto config = makeConfig(topic);
  GetPoseWithCovarianceFromTopic behavior{ "GetPoseWithCovarianceFromTopic", config, spun.context };
  EXPECT_EQ(tickUntilDone(behavior), BT::NodeStatus::FAILURE);
}

TEST(GetPoseWithCovarianceFromTopic, FailsOnANegativeTimeout)
{
  SpunContext spun;
  const std::string topic = "/localization_behaviors_test/negative_timeout_pose";
  const auto publisher = spun.node->create_publisher<PoseWithCovarianceStamped>(topic, latchedQoS());
  publisher->publish(makeMessage(0.04, 0.09, 0.01));
  auto config = makeConfig(topic);
  config.input_ports["message_timeout_sec"] = "-1.0";
  GetPoseWithCovarianceFromTopic behavior{ "GetPoseWithCovarianceFromTopic", config, spun.context };
  EXPECT_EQ(tickUntilDone(behavior), BT::NodeStatus::FAILURE);
}

TEST(GetPoseWithCovarianceFromTopic, HaltStopsTheWaitPromptly)
{
  SpunContext spun;
  auto config = makeConfig("/localization_behaviors_test/halted_pose");
  config.input_ports["message_timeout_sec"] = "30.0";
  GetPoseWithCovarianceFromTopic behavior{ "GetPoseWithCovarianceFromTopic", config, spun.context };
  ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::RUNNING);

  const auto start = std::chrono::steady_clock::now();
  behavior.haltNode();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds{ 2 });
  EXPECT_EQ(behavior.status(), BT::NodeStatus::IDLE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
