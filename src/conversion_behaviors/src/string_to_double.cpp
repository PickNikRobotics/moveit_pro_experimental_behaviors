// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "conversion_behaviors/string_to_double.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

namespace
{
constexpr auto kPortIDInput = "string_in";
constexpr auto kPortIDOutput = "double_out";
}  // namespace

namespace conversion_behaviors
{

StringToDouble::StringToDouble(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList StringToDouble::providedPorts()
{
  return BT::PortsList({ BT::InputPort<std::string>(kPortIDInput, "{string_in}", "The input string"),
                         BT::OutputPort<double>(kPortIDOutput, "{double_out}", "The output double.") });
}

BT::KeyValueVector StringToDouble::metadata()
{
  return { { "description", "Creates a double from the string provided" }, { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus StringToDouble::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDInput));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [string_in] = ports.value();

  try
  {
    double double_value = std::stod(string_in);
    setOutput(kPortIDOutput, double_value);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::invalid_argument&)
  {
    spdlog::warn("Invalid double string: '{}'", string_in);
    return BT::NodeStatus::FAILURE;
  }
  catch (const std::out_of_range&)
  {
    spdlog::warn("Double out of range: '{}'", string_in);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace conversion_behaviors
