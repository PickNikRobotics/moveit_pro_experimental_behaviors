// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "conversion_behaviors/create_vector_of_doubles.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <string>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

namespace
{
constexpr auto kPortIDInput = "input";
constexpr auto kPortIDOutput = "output";
}  // namespace

namespace conversion_behaviors
{

CreateVectorOfDoubles::CreateVectorOfDoubles(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList CreateVectorOfDoubles::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<std::vector<double>>(kPortIDInput, "{input}", "The input vector of doubles"),
        BT::OutputPort<std::vector<double>>(kPortIDOutput, "{output}", "The output vector of doubles.") });
}

BT::KeyValueVector CreateVectorOfDoubles::metadata()
{
  return { { "description", "Creates a vector of doubles from the input port" },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus CreateVectorOfDoubles::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::vector<double>>(kPortIDInput));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& input = std::get<0>(ports.value());

  setOutput(kPortIDOutput, input);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace conversion_behaviors
