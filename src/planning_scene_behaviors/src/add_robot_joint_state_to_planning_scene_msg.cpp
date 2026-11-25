#include "planning_scene_behaviors/add_robot_joint_state_to_planning_scene_msg.hpp"

namespace planning_scene_behaviors
{

AddRobotJointStateToPlanningSceneMsg::AddRobotJointStateToPlanningSceneMsg(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList AddRobotJointStateToPlanningSceneMsg::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::PlanningScene>("input_planning_scene", "Input planning scene message"),
           BT::InputPort<moveit_msgs::msg::RobotState>("robot_joint_state",
                                                       "Robot joint state to add to planning scene"),
           BT::OutputPort<moveit_msgs::msg::PlanningScene>("output_planning_scene",
                                                           "Output planning scene with updated robot state") };
}

BT::KeyValueVector AddRobotJointStateToPlanningSceneMsg::metadata()
{
  return { { "description", "Updates a planning scene message with robot joint state values" },
           { "subcategory", "Planning Scene" } };
}

BT::NodeStatus AddRobotJointStateToPlanningSceneMsg::tick()
{
  // Get input planning scene
  const auto input_planning_scene = getInput<moveit_msgs::msg::PlanningScene>("input_planning_scene");
  if (!input_planning_scene)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [input_planning_scene]: " +
                                                                 input_planning_scene.error());
    return BT::NodeStatus::FAILURE;
  }

  // Get robot joint state
  const auto robot_joint_state = getInput<moveit_msgs::msg::RobotState>("robot_joint_state");
  if (!robot_joint_state)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [robot_joint_state]: " +
                                                                 robot_joint_state.error());
    return BT::NodeStatus::FAILURE;
  }

  // Create output planning scene by copying the input
  moveit_msgs::msg::PlanningScene output_planning_scene = input_planning_scene.value();

  // Update the robot state with the joint values from robot_joint_state
  output_planning_scene.robot_state.joint_state = robot_joint_state->joint_state;
  output_planning_scene.robot_state.multi_dof_joint_state = robot_joint_state->multi_dof_joint_state;

  // Set the is_diff flag to true since we're modifying the robot state
  output_planning_scene.robot_state.is_diff = true;

  // Set output
  setOutput("output_planning_scene", output_planning_scene);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace planning_scene_behaviors
