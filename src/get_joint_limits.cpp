// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_joint_limits.hpp>

#include <string>

#include <moveit_pro_base/robot_model/joint_model.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>

namespace experimental_behaviors
{

GetJointLimits::GetJointLimits(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetJointLimits::providedPorts()
{
  return { BT::InputPort<std::string>("joint_name", "Name of the (single-DOF) joint to read position limits for."),
           BT::OutputPort<double>("min_position", "Lower position limit from the robot model (URDF)."),
           BT::OutputPort<double>("max_position", "Upper position limit from the robot model (URDF).") };
}

BT::KeyValueVector GetJointLimits::metadata()
{
  return { { "description", "Reads a single-DOF joint's position limits (min/max) from the loaded robot model "
                            "(robot_description), so trees obtain limits from the URDF instead of a static YAML." },
           { "subcategory", "Robot State" } };
}

BT::NodeStatus GetJointLimits::tick()
{
  const auto joint_name = getInput<std::string>("joint_name");
  if (!joint_name)
  {
    shared_resources_->logger->publishWarnMessage(name(),
                                                  "Failed to get required input [joint_name]: " + joint_name.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& robot_model = shared_resources_->robot_model;
  if (!robot_model)
  {
    shared_resources_->logger->publishWarnMessage(
        name(), "Robot model is not available (the BehaviorContext must be created with load_robot_model=true).");
    return BT::NodeStatus::FAILURE;
  }

  const auto* joint_model = robot_model->getJointModel(joint_name.value());
  if (joint_model == nullptr)
  {
    shared_resources_->logger->publishWarnMessage(name(),
                                                  "Joint '" + joint_name.value() + "' not found in the robot model.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& bounds = joint_model->getVariableBounds();
  if (bounds.size() != 1)
  {
    shared_resources_->logger->publishWarnMessage(
        name(), "Joint '" + joint_name.value() + "' is not single-DOF (has " + std::to_string(bounds.size()) +
                    " variables); GetJointLimits supports single-DOF joints only.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& variable_bounds = bounds.front();
  if (!variable_bounds.position_bounded_)
  {
    shared_resources_->logger->publishWarnMessage(name(), "Joint '" + joint_name.value() +
                                                              "' has no position limits in the robot model.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("min_position", variable_bounds.min_position_);
  setOutput("max_position", variable_bounds.max_position_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace experimental_behaviors
