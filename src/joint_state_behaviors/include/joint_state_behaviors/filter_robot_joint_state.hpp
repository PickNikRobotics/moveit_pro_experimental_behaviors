#pragma once

#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>

namespace joint_state_behaviors
{

/**
 * @brief Filters a JointState message to only include joints from a specific planning group.
 *
 * This behavior takes a full robot JointState and extracts only the joints that belong
 * to the specified planning group, preserving their order and associated data.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * planning_group_name | input | std::string
 * input_robot_joint_state | input | moveit_msgs::msg::RobotState
 * output_robot_joint_state | output | moveit_msgs::msg::RobotState
 */
class FilterRobotJointState : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  FilterRobotJointState(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  /**
   * @brief Implementation of the async work that will be executed in a separate thread.
   * @details This method contains the main logic for filtering the joint state.
   * @return tl::expected<bool, std::string> - Returns true on success, or an error message on failure.
   */
  tl::expected<bool, std::string> doWork() override;

  /**
   * @brief Gets the shared future for the async operation.
   * @return Reference to the shared future.
   */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Shared future for the async operation result. */
  std::shared_future<tl::expected<bool, std::string>> future_;
};

}  // namespace joint_state_behaviors
