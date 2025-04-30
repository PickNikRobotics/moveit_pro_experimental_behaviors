#include "experimental_behaviors/get_pose_stamped_from_topic.hpp"

#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace experimental_behaviors
{

namespace
{
constexpr auto kPortIDTimeoutDuration = "timeout_sec";
}  // namespace

GetPoseStampedFromTopic::GetPoseStampedFromTopic(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<geometry_msgs::msg::PoseStamped>(name, config,
                                                                                               shared_resources)
{
}

BT::PortsList GetPoseStampedFromTopic::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<std::string>(kPortIDTopicName, "", "PoseStamped topic name."),
      BT::InputPort<double>(kPortIDTimeoutDuration, -1.0,
                            "Maximum duration in seconds to wait for pose message to be published before "
                            "failing. Set to -1.0 to wait indefinitely."),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDMessageOut, "PoseStamped message."),
  });
}

tl::expected<std::chrono::duration<double>, std::string> GetPoseStampedFromTopic::getWaitForMessageTimeout()
{
  const auto port = moveit_studio::behaviors::getRequiredInputs(getInput<double>(kPortIDTimeoutDuration));

  // Check that input data ports were set.
  if (!port.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + port.error());
  }
  const auto& [timeout] = port.value();
  if (timeout == 0.0)
  {
    return tl::make_unexpected("Timeout value is 0.0s.");
  }
  return std::chrono::duration<double>{ timeout };
}
}  // namespace experimental_behaviors

template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<geometry_msgs::msg::PoseStamped>;
