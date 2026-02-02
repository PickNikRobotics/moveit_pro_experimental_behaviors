// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/access_interface_value_from_group.hpp>

#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionAccessInterfaceValueFromGroup = R"(
                <p>
                     Accesses a control_msgs::msg::InterfaceValue message from a group and writes it to the blackboard.
                </p>
            )";
constexpr auto kPortIDInterfaceName = "interface_name";
constexpr auto kPortIDValue = "value";
constexpr auto kPortIDInterfaceValue = "dynamic_interface_value";
}  // namespace

namespace experimental_behaviors
{
AccessInterfaceValueFromGroup::AccessInterfaceValueFromGroup(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList AccessInterfaceValueFromGroup::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDInterfaceName,
                                              "Interface name."),
           BT::OutputPort<double>(kPortIDValue,
                                              "value of interface listed as `interface_name`."),
           BT::InputPort<control_msgs::msg::InterfaceValue>(kPortIDInterfaceValue, "{interface_value}",
                                                            "The control_msgs::msg::InterfaceValue message.") };
}

BT::KeyValueVector AccessInterfaceValueFromGroup::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "Conversions" }, { moveit_studio::behaviors::kDescriptionMetadataKey, kDescriptionAccessInterfaceValueFromGroup } };
}

BT::NodeStatus AccessInterfaceValueFromGroup::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<std::string>(kPortIDInterfaceName),
                                       getInput<control_msgs::msg::InterfaceValue>(kPortIDInterfaceValue));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [interface_name, interface] = ports.value();
  
  for (size_t i = 0; i < interface.interface_names.size(); ++i) {
    if (interface.interface_names[i] == interface_name) {
      if (i >= interface.values.size()) {
        shared_resources_->logger->publishFailureMessage(name(), "Interface value index out of range");
        return BT::NodeStatus::FAILURE;
      }
      setOutput(kPortIDValue, interface.values[i]);
      return BT::NodeStatus::SUCCESS;
    }
  }
  shared_resources_->logger->publishFailureMessage(name(), "Interface name not found in interface value message");
  return BT::NodeStatus::FAILURE;
}

}  // experimental_behaviors
