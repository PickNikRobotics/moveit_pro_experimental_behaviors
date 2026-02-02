// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_interface_value_from_group.hpp>

#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionGetInterfaceValueFromGroup = R"(
                <p>
                    Gets an InterfaceValue from a DynamicInterfaceGroupValues message by interface group name.
                </p>
            )";
constexpr auto kPortIDGroupValues = "group_values";
constexpr auto kPortIDInterfaceGroupName = "interface_group_name";
constexpr auto kPortIDInterfaceValue = "interface_value";
}  // namespace

namespace experimental_behaviors
{
GetInterfaceValueFromGroup::GetInterfaceValueFromGroup(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetInterfaceValueFromGroup::providedPorts()
{
  return { BT::InputPort<control_msgs::msg::DynamicInterfaceGroupValues>(kPortIDGroupValues,
                                                            "The control_msgs::msg::DynamicInterfaceGroupValues message."),
           BT::InputPort<std::string>(kPortIDInterfaceGroupName,
                                      "The name of the interface group to retrieve."),
           BT::OutputPort<control_msgs::msg::InterfaceValue>(kPortIDInterfaceValue, "{interface_value}",
                                                            "The control_msgs::msg::InterfaceValue message.") };
}

BT::KeyValueVector GetInterfaceValueFromGroup::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "Conversions" }, 
           { moveit_studio::behaviors::kDescriptionMetadataKey, kDescriptionGetInterfaceValueFromGroup } };
}

BT::NodeStatus GetInterfaceValueFromGroup::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<control_msgs::msg::DynamicInterfaceGroupValues>(kPortIDGroupValues),
      getInput<std::string>(kPortIDInterfaceGroupName));
  
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [group_values, interface_group_name] = ports.value();

  // Search for the interface group name
  auto it = std::find(group_values.interface_groups.begin(), 
                      group_values.interface_groups.end(), 
                      interface_group_name);

  if (it == group_values.interface_groups.end())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Interface group '" + interface_group_name + "' not found in DynamicInterfaceGroupValues message");
    return BT::NodeStatus::FAILURE;
  }

  // Get the index of the found group
  const size_t index = std::distance(group_values.interface_groups.begin(), it);

  // Check that the index is valid for the interface_values vector
  if (index >= group_values.interface_values.size())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Index mismatch: interface_groups and interface_values have different sizes");
    return BT::NodeStatus::FAILURE;
  }

  // Get the corresponding InterfaceValue
  const auto& interface_value = group_values.interface_values[index];

  setOutput(kPortIDInterfaceValue, interface_value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace experimental_behaviors
