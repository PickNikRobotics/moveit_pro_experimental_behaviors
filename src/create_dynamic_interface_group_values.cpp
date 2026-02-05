// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/create_dynamic_interface_group_values.hpp>

#include <control_msgs/msg/interface_value.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionCreateDynamicInterfaceGroupValues = R"(
                <p>
                    Creates a control_msgs::msg::DynamicInterfaceGroupValues message and writes it to the blackboard.
                </p>
            )";
constexpr auto kPortIDReferenceFrame = "reference_frame";
constexpr auto kPortIDInterfaceGroups = "interface_groups";
constexpr auto kPortIDInterfaceValues = "interface_values";
constexpr auto kPortIDGroupValues = "group_values";
}  // namespace

namespace experimental_behaviors
{
CreateDynamicInterfaceGroupValues::CreateDynamicInterfaceGroupValues(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CreateDynamicInterfaceGroupValues::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDReferenceFrame, "world",
                               "The reference frame of the control_msgs::msg::DynamicInterfaceGroupValues message."),
    BT::InputPort<std::vector<std::string>>(kPortIDInterfaceGroups,
                                            "List of interface group names , e.g. `flange_analog_IOs; flange_vacuum`."),
    BT::InputPort<std::vector<control_msgs::msg::InterfaceValue>>(
        kPortIDInterfaceValues, "List of corresponding InterfaceValue for each group listed in `interface_groups`."),
    BT::OutputPort<control_msgs::msg::DynamicInterfaceGroupValues>(
        kPortIDGroupValues, "{group_values}", "The control_msgs::msg::DynamicInterfaceGroupValues message.")
  };
}

BT::KeyValueVector CreateDynamicInterfaceGroupValues::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Conversions" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionCreateDynamicInterfaceGroupValues } };
}

BT::NodeStatus CreateDynamicInterfaceGroupValues::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::string>(kPortIDReferenceFrame), getInput<std::vector<std::string>>(kPortIDInterfaceGroups),
      getInput<std::vector<control_msgs::msg::InterfaceValue>>(kPortIDInterfaceValues));
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [reference_frame, interface_groups, interface_values] = ports.value();

  if (interface_groups.size() != interface_values.size())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "`interface_groups` and `interface_values` must contain the same number of elements");
    return BT::NodeStatus::FAILURE;
  }

  if (interface_groups.empty() || interface_values.empty())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "`interface_groups` and `interface_values` must contain the same number of elements and not be empty");
    return BT::NodeStatus::FAILURE;
  }

  control_msgs::msg::DynamicInterfaceGroupValues group_values_msg;

  group_values_msg.header.frame_id = reference_frame;
  group_values_msg.header.stamp = shared_resources_->node->now();

  group_values_msg.interface_groups = interface_groups;

  group_values_msg.interface_values = interface_values;

  setOutput(kPortIDGroupValues, group_values_msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace experimental_behaviors
