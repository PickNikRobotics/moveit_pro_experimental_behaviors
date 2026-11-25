#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace trajectory_behaviors
{

class FilterABAJointTrajectory : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  FilterABAJointTrajectory(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace trajectory_behaviors
