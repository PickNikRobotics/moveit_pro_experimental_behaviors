// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/localization_behaviors/call_empty_service_by_name.hpp>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortServiceName = "service_name";
constexpr auto kPortResponseTimeout = "response_timeout";
constexpr auto kPortServerTimeout = "wait_for_server_available_timeout";
constexpr auto kDefaultTimeoutSeconds = 3.0;

constexpr auto kDescription = R"(
    <p>Sends a request to a <code>std_srvs/srv/Empty</code> service and waits for the response.</p>
    <p>An Empty response has no success field, so any response is SUCCESS. FAILURE if no server with that name
    appears before <code>wait_for_server_available_timeout</code>, or the response does not arrive before
    <code>response_timeout</code> (negative waits forever).</p>
    <p>Newer MoveIt Pro releases include the same Behavior as <code>CallEmptyService</code>.</p>
)";

tl::expected<std::chrono::duration<double>, std::string> toDuration(const BT::Expected<double>& seconds)
{
  if (!seconds)
  {
    return tl::make_unexpected(seconds.error());
  }
  return std::chrono::duration<double>{ seconds.value() };
}
}  // namespace

namespace experimental_behaviors
{
CallEmptyServiceByName::CallEmptyServiceByName(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<Empty>(name, config, shared_resources)
{
}

CallEmptyServiceByName::CallEmptyServiceByName(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Empty>> client_interface)
  : ServiceClientBehaviorBase<Empty>(name, config, shared_resources, std::move(client_interface))
{
}

BT::PortsList CallEmptyServiceByName::providedPorts()
{
  return { BT::InputPort<std::string>(kPortServiceName, "Name of the std_srvs/srv/Empty service to call."),
           BT::InputPort<double>(kPortResponseTimeout, kDefaultTimeoutSeconds,
                                 "Seconds to wait for the response. Negative waits forever."),
           BT::InputPort<double>(kPortServerTimeout, kDefaultTimeoutSeconds,
                                 "Seconds to wait for the service server to become available.") };
}

BT::KeyValueVector CallEmptyServiceByName::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Control Flow" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

tl::expected<std::string, std::string> CallEmptyServiceByName::getServiceName()
{
  const auto service_name = getInput<std::string>(kPortServiceName);
  if (!service_name)
  {
    return tl::make_unexpected(service_name.error());
  }
  return service_name.value();
}

tl::expected<std::chrono::duration<double>, std::string> CallEmptyServiceByName::getResponseTimeout()
{
  return toDuration(getInput<double>(kPortResponseTimeout));
}

tl::expected<std::chrono::duration<double>, std::string> CallEmptyServiceByName::getWaitForServerAvailableTimeout()
{
  return toDuration(getInput<double>(kPortServerTimeout));
}

tl::expected<Empty::Request, std::string> CallEmptyServiceByName::createRequest()
{
  return Empty::Request{};
}
}  // namespace experimental_behaviors

template class moveit_pro::behaviors::ServiceClientBehaviorBase<std_srvs::srv::Empty>;
template class moveit_pro::behaviors::ClientInterfaceBase<std_srvs::srv::Empty>;
