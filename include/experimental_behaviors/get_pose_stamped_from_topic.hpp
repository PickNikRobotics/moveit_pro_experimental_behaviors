#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/get_message_from_topic.hpp>

namespace experimental_behaviors
{
/**
 * @brief Subscribe to PoseStamped message and write received message to the blackboard.
 */
class GetPoseStampedFromTopic final
  : public moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<geometry_msgs::msg::PoseStamped>
{
public:
  GetPoseStampedFromTopic(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

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
}  // namespace experimental_behaviors
