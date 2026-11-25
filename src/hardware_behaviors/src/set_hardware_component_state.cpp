// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "hardware_behaviors/set_hardware_component_state.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/service_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <string>

namespace
{
inline constexpr auto kDescriptionSetHardwareComponentState = R"(
  <p>Drives a <code>ros2_control</code> hardware component through a lifecycle transition by calling the controller
  manager's <code>set_hardware_component_state</code> service. This is the Behavior equivalent of
  <code>ros2 control set_hardware_component_state &lt;name&gt; &lt;state&gt;</code>.</p>
  <p>Use it on recovery paths where the hardware itself went down rather than just its controllers: a driver error
  transitions the component out of the active state, and <code>SwitchController</code> alone cannot bring it back. The
  controller manager works out the transition chain, so an unconfigured component can be taken straight to
  <code>active</code> in one call.</p>
  <p>Requesting the state a component is already in reports failure, so wrap a defensive <code>inactive</code> step in
  a <code>ForceSuccess</code> decorator.</p>
)";

constexpr auto kPortServiceName = "service_name";
constexpr auto kPortHardwareName = "hardware_name";
constexpr auto kPortTargetState = "target_state";
constexpr auto kPortTimeout = "timeout";

constexpr auto kDefaultServiceName = "/controller_manager/set_hardware_component_state";

/**
 * @brief Convert a target state name to the matching lifecycle state.
 * @details Both the id and the label are populated: the controller manager matches on the id, and the label makes the
 * request readable in logs and service introspection.
 * @return True if the name was recognized, in which case @a state is populated.
 */
bool parseTargetState(std::string name, lifecycle_msgs::msg::State& state)
{
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });

  if (name == "active" || name == "activate")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    state.label = "active";
    return true;
  }
  if (name == "inactive" || name == "deactivate")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    state.label = "inactive";
    return true;
  }
  if (name == "unconfigured" || name == "cleanup")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    state.label = "unconfigured";
    return true;
  }
  if (name == "finalized" || name == "shutdown")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    state.label = "finalized";
    return true;
  }
  return false;
}
}  // namespace

namespace hardware_behaviors
{
SetHardwareComponentState::SetHardwareComponentState(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ServiceClientBehaviorBase<SetHardwareComponentStateSrv>(name, config, shared_resources)
{
}

BT::PortsList SetHardwareComponentState::providedPorts()
{
  return { BT::InputPort<std::string>(kPortServiceName, kDefaultServiceName,
                                      "Name of the controller manager's set_hardware_component_state service."),
           BT::InputPort<std::string>(kPortHardwareName, "",
                                      "Name of the hardware component to transition, as declared by the <ros2_control "
                                      "name=\"...\"> tag in the URDF. List the available names with "
                                      "`ros2 control list_hardware_components`."),
           BT::InputPort<std::string>(kPortTargetState, "active",
                                      "Target lifecycle state: active, inactive, unconfigured, or finalized."),
           BT::InputPort<double>(kPortTimeout, 10.0,
                                 "Seconds to wait for the service response before failing. A negative value waits "
                                 "indefinitely.") };
}

BT::KeyValueVector SetHardwareComponentState::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Controls" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSetHardwareComponentState } };
}

tl::expected<std::string, std::string> SetHardwareComponentState::getServiceName()
{
  const auto service_name = getInput<std::string>(kPortServiceName);
  if (!service_name.has_value())
  {
    return tl::make_unexpected(
        fmt::format("Failed to get required value from input port '{}': {}", kPortServiceName, service_name.error()));
  }
  if (service_name.value().empty())
  {
    return tl::make_unexpected(fmt::format("The '{}' port must contain a valid service name.", kPortServiceName));
  }
  return service_name.value();
}

tl::expected<SetHardwareComponentStateSrv::Request, std::string> SetHardwareComponentState::createRequest()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortHardwareName),
                                                              getInput<std::string>(kPortTargetState));
  if (!ports.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to get required values from input data ports: {}", ports.error()));
  }
  const auto& [hardware_name, target_state] = ports.value();

  if (hardware_name.empty())
  {
    return tl::make_unexpected(fmt::format("The '{}' port must contain a hardware component name.", kPortHardwareName));
  }

  SetHardwareComponentStateSrv::Request request;
  request.name = hardware_name;
  if (!parseTargetState(target_state, request.target_state))
  {
    return tl::make_unexpected(
        fmt::format("Invalid '{}' value '{}'. Use one of: active, inactive, unconfigured, finalized.", kPortTargetState,
                    target_state));
  }
  return request;
}

tl::expected<bool, std::string>
SetHardwareComponentState::processResponse(const SetHardwareComponentStateSrv::Response& response)
{
  if (!response.ok)
  {
    // The service reports no error string, so surface the state the component actually ended up in instead.
    return tl::make_unexpected(
        fmt::format("The controller manager rejected the hardware component state transition. The component is in the "
                    "'{}' state.",
                    response.state.label));
  }
  return true;
}

tl::expected<std::chrono::duration<double>, std::string> SetHardwareComponentState::getResponseTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected(
        fmt::format("Failed to get required value from input port '{}': {}", kPortTimeout, ports.error()));
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}
}  // namespace hardware_behaviors

// This specializes ServiceClientBehaviorBase for the SetHardwareComponentState service.
template class moveit_pro::behaviors::ServiceClientBehaviorBase<hardware_behaviors::SetHardwareComponentStateSrv>;
