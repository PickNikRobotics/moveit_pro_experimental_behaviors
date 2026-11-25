#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace planning_scene_behaviors
{

/**
 * @brief Adds robot joint state to a planning scene message.
 *
 * This behavior takes a PlanningScene message and a RobotState, and updates
 * the planning scene's robot_state with the joint values from the RobotState.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * input_planning_scene | input | moveit_msgs::msg::PlanningScene
 * robot_joint_state | input | moveit_msgs::msg::RobotState
 * output_planning_scene | output | moveit_msgs::msg::PlanningScene
 */
class AddRobotJointStateToPlanningSceneMsg : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  AddRobotJointStateToPlanningSceneMsg(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace planning_scene_behaviors
