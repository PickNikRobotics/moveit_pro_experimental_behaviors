#include "ros_messaging_behaviors/get_string_from_topic.hpp"

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace ros_messaging_behaviors
{

GetStringFromTopic::GetStringFromTopic(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<std_msgs::msg::String>(name, config, shared_resources)
{
}

BT::PortsList GetStringFromTopic::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<std::string>(kPortIDTopicName, "", "String topic name."),
      BT::InputPort<double>(kPortIDTimeoutDuration, -1.0,
                            "Maximum duration in seconds to wait for string message to be published before "
                            "failing. Set to -1.0 to wait indefinitely."),
      BT::OutputPort<std_msgs::msg::String>(kPortIDMessageOut, "String message."),
  });
}

tl::expected<std::chrono::duration<double>, std::string> GetStringFromTopic::getWaitForMessageTimeout()
{
  const auto port = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortIDTimeoutDuration));

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
}  // namespace ros_messaging_behaviors

template class moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<std_msgs::msg::String>;
