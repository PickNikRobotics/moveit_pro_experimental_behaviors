// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "experimental_behaviors/get_dynamic_interface_group_values.hpp"

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace experimental_behaviors
{

GetDynamicInterfaceGroupValues::GetDynamicInterfaceGroupValues(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<control_msgs::msg::DynamicInterfaceGroupValues>(
        name, config, shared_resources)
{
}

BT::PortsList GetDynamicInterfaceGroupValues::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<std::string>(kPortIDTopicName, "", "DynamicInterfaceGroupValues topic name."),
      BT::InputPort<double>(kPortIDTimeoutDuration, -1.0,
                            "Maximum duration in seconds to wait for dynamic interface group values message to be "
                            "published before "
                            "failing. Set to -1.0 to wait indefinitely."),
      BT::OutputPort<control_msgs::msg::DynamicInterfaceGroupValues>(kPortIDMessageOut,
                                                                     "DynamicInterfaceGroupValues message."),
  });
}

tl::expected<std::chrono::duration<double>, std::string> GetDynamicInterfaceGroupValues::getWaitForMessageTimeout()
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
}  // namespace experimental_behaviors
