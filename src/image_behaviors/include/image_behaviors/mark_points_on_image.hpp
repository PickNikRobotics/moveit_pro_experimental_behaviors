// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace image_behaviors
{

class MarkPointsOnImage final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  MarkPointsOnImage(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::string current_topic_;
};

}  // namespace image_behaviors
