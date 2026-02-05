#pragma once

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <moveit_pro_behavior_interface/get_message_from_topic.hpp>

namespace experimental_behaviors
{
/**
 * @brief Subscribe to DynamicJointState message and write received message to the blackboard.
 */
class GetDynamicInterfaceGroupValues final
  : public moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<control_msgs::msg::DynamicInterfaceGroupValues>
{
public:
  GetDynamicInterfaceGroupValues(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

private:
  tl::expected<std::chrono::duration<double>, std::string> getWaitForMessageTimeout() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace experimental_behaviors
