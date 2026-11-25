// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "conversion_behaviors/double_to_string.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <iomanip>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

namespace
{
constexpr auto kPortIDInput = "double_in";
constexpr auto kPortIDOutput = "string_out";
}  // namespace

namespace conversion_behaviors
{

DoubleToString::DoubleToString(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList DoubleToString::providedPorts()
{
  return BT::PortsList({ BT::InputPort<double>(kPortIDInput, "{double_in}", "The input double"),
                         BT::OutputPort<std::string>(kPortIDOutput, "{string_out}", "The output string.") });
}

BT::KeyValueVector DoubleToString::metadata()
{
  return { { "description", "Creates a string from the double provided" }, { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus DoubleToString::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortIDInput));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [double_in] = ports.value();

  std::stringstream out;
  out << std::fixed << std::setprecision(3) << double_in;
  const std::string result_str = out.str();
  setOutput(kPortIDOutput, result_str);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace conversion_behaviors
