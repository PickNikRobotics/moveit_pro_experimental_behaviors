// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <memory>
#include <string>

namespace experimental_behaviors::test
{
/// Loads this package's Behavior plugin into a factory and runs small trees against one blackboard.
class JsonBehaviorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("json_behavior_test");
    context_ = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_, false);
    loader_ = class_loader_.createUniqueInstance("experimental_behaviors::ExperimentalBehaviorsLoader");
    loader_->registerBehaviors(factory_, context_);
  }

  /// Builds a tree from the XML of the main tree's body and keeps it alive until the next call.
  BT::Tree& createTree(const std::string& body)
  {
    const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">)" + body +
                            "</BehaviorTree></root>";
    tree_ = factory_.createTreeFromText(xml, blackboard_);
    return tree_;
  }

  BT::NodeStatus run(const std::string& body)
  {
    return createTree(body).tickWhileRunning();
  }

  /// Returns, and clears, the failure messages the Behaviors published.
  std::string errors()
  {
    return context_->logger->consumeErrorLogBuffer();
  }

  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader_{
    "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase"
  };
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  pluginlib::UniquePtr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_ = BT::Blackboard::create();
  BT::Tree tree_;
};
}  // namespace experimental_behaviors::test
