#pragma once

#include <moveit_pro_behavior_interface/get_message_from_topic.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_messaging_behaviors
{
/**
 * @brief Subscribe to String message and write received message to the blackboard.
 */
class GetStringFromTopic final : public moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<std_msgs::msg::String>
{
public:
  GetStringFromTopic(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  tl::expected<std::chrono::duration<double>, std::string> getWaitForMessageTimeout() override;

private:
  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace ros_messaging_behaviors
