// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

// PublishPoseStamped must publish the blackboard pose unchanged and latch it, so that a subscriber
// that joins after the tick still gets it.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_messaging_behaviors/publish_pose_stamped.hpp"

namespace
{
constexpr auto kTopic = "/test_published_pose";

TEST(PublishPoseStamped, PublishesTheBlackboardPoseUnchanged)
{
  auto node = std::make_shared<rclcpp::Node>("test_publish_pose_stamped");
  // No TF spin thread: this test needs no transforms.
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);

  // A distinct frame, stamp and orientation, so a field the Behavior drops or defaults shows.
  geometry_msgs::msg::PoseStamped sent;
  sent.header.frame_id = "base_link";
  sent.header.stamp.sec = 4242;
  sent.pose.position.x = 0.31;
  sent.pose.position.y = -0.17;
  sent.pose.position.z = 0.55;
  sent.pose.orientation.x = 0.5;
  sent.pose.orientation.y = -0.5;
  sent.pose.orientation.z = 0.5;
  sent.pose.orientation.w = 0.5;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("target_pose", sent);

  BT::NodeConfiguration config;
  config.blackboard = blackboard;
  config.input_ports["pose"] = "{target_pose}";
  config.input_ports["topic_name"] = kTopic;

  ros_messaging_behaviors::PublishPoseStamped behavior("test", config, shared_resources);
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::SUCCESS);

  // Subscribe after the tick on purpose: the latched sample must reach a late subscriber.
  geometry_msgs::msg::PoseStamped::SharedPtr received;
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      kTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [&received](geometry_msgs::msg::PoseStamped::SharedPtr msg) { received = msg; });
  for (int i = 0; i < 200 && !received; ++i)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(received) << "no latched sample for the late subscriber";
  EXPECT_EQ(received->header.frame_id, sent.header.frame_id);
  EXPECT_EQ(received->header.stamp.sec, sent.header.stamp.sec);
  EXPECT_DOUBLE_EQ(received->pose.position.x, sent.pose.position.x);
  EXPECT_DOUBLE_EQ(received->pose.position.y, sent.pose.position.y);
  EXPECT_DOUBLE_EQ(received->pose.position.z, sent.pose.position.z);
  EXPECT_DOUBLE_EQ(received->pose.orientation.x, sent.pose.orientation.x);
  EXPECT_DOUBLE_EQ(received->pose.orientation.y, sent.pose.orientation.y);
  EXPECT_DOUBLE_EQ(received->pose.orientation.z, sent.pose.orientation.z);
  EXPECT_DOUBLE_EQ(received->pose.orientation.w, sent.pose.orientation.w);
}

TEST(PublishPoseStamped, MissingPortsFailWithoutPublishing)
{
  auto node = std::make_shared<rclcpp::Node>("test_publish_pose_stamped_missing");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);

  bool got_message = false;
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      kTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [&got_message](geometry_msgs::msg::PoseStamped::SharedPtr) { got_message = true; });

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  ros_messaging_behaviors::PublishPoseStamped behavior("test", config, shared_resources);
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::FAILURE);

  rclcpp::spin_some(node);
  EXPECT_FALSE(got_message);
}
}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
