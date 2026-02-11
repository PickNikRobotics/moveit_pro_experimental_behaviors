// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/create_interface_value.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionCreateInterfaceValue = R"(
                <p>
                    Creates a control_msgs::msg::InterfaceValue message and writes it to the blackboard.
                </p>
            )";
constexpr auto kPortIDInterfaceNames = "interface_names";
constexpr auto kPortIDValues = "values";
constexpr auto kPortIDInterfaceValue = "interface_value";
}  // namespace

namespace experimental_behaviors
{
CreateInterfaceValue::CreateInterfaceValue(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CreateInterfaceValue::providedPorts()
{
  return { BT::InputPort<std::vector<std::string>>(kPortIDInterfaceNames, "List of interface names."),
           BT::InputPort<std::vector<double>>(kPortIDValues, "List of corresponding value for each interface listed in "
                                                             "`interface_names`."),
           BT::OutputPort<control_msgs::msg::InterfaceValue>(kPortIDInterfaceValue, "{interface_value}",
                                                             "The control_msgs::msg::InterfaceValue message.") };
}

BT::KeyValueVector CreateInterfaceValue::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Conversions" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionCreateInterfaceValue } };
}

BT::NodeStatus CreateInterfaceValue::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::vector<std::string>>(kPortIDInterfaceNames),
                                                              getInput<std::vector<double>>(kPortIDValues));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [interface_names, values] = ports.value();

  if (interface_names.size() != values.size())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "`interface_names` and `values` must contain the same number of elements");
    return BT::NodeStatus::FAILURE;
  }

  if (interface_names.empty() || values.empty())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "`interface_names` and `values` must contain the same number of elements and not be empty");
    return BT::NodeStatus::FAILURE;
  }

  control_msgs::msg::InterfaceValue interface_value_msg;

  interface_value_msg.interface_names = interface_names;

  interface_value_msg.values = values;

  setOutput(kPortIDInterfaceValue, interface_value_msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace experimental_behaviors
