// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace experimental_behaviors::test
{
/** @brief A behavior node spun on its own executor thread, as the Objective Server spins it. */
struct SpunContext
{
  SpunContext() : node{ std::make_shared<rclcpp::Node>("localization_behaviors_test") }
  {
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);
    executor.add_node(node);
    spinner = std::thread{ [this] {
      executor.spin();
      stopped = true;
    } };
  }
  ~SpunContext()
  {
    // A cancel sent before spin() starts is lost, so repeat it until the spinner returns.
    while (!stopped)
    {
      executor.cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
    }
    spinner.join();
  }
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::atomic<bool> stopped{ false };
  std::thread spinner;
};

/** @brief Ticks until the node leaves RUNNING or the timeout passes. */
inline BT::NodeStatus tickUntilDone(BT::StatefulActionNode& behavior,
                                    std::chrono::milliseconds timeout = std::chrono::seconds{ 15 })
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto status = behavior.executeTick();
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
    status = behavior.executeTick();
  }
  return status;
}

inline BT::NodeConfiguration makeConfig()
{
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  return config;
}
}  // namespace experimental_behaviors::test
