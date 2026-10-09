// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/send_json_udp.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <experimental_behaviors/json_behaviors/udp_socket.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionSendJsonUdp = R"(
                <p>
                    Sends a JSON document to another system as one UDP datagram. Use it for frequent,
                    small status messages where a lost message does no harm, for example a heartbeat to a
                    dashboard.
                </p>
                <p>
                    <code>host</code> is an IPv4 or IPv6 address or a host name. The payload must be valid
                    JSON and fit in one datagram. UDP does not confirm delivery: SUCCESS means the datagram
                    was sent, not that it arrived. Use <code>SendHttpRequest</code> when delivery matters.
                </p>
            )";

constexpr auto kPortIDHost = "host";
constexpr auto kPortIDPort = "port";
constexpr auto kPortIDPayload = "payload";
}  // namespace

namespace experimental_behaviors
{
SendJsonUdp::SendJsonUdp(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SendJsonUdp::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDHost, "127.0.0.1", "Destination IPv4 or IPv6 address, or host name."),
           BT::InputPort<int>(kPortIDPort, "Destination UDP port."),
           BT::InputPort<std::string>(kPortIDPayload, "{json}", "JSON document to send, as JSON text.") };
}

BT::KeyValueVector SendJsonUdp::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSendJsonUdp } };
}

BT::NodeStatus SendJsonUdp::tick()
{
  const auto host = getInput<std::string>(kPortIDHost);
  const auto port = getInput<int>(kPortIDPort);
  const auto payload = json_utils::getJsonText(*this, kPortIDPayload);
  if (!host || !port || !payload)
  {
    const std::string error = !host ? host.error() : (!port ? port.error() : payload.error());
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " + error);
    return BT::NodeStatus::FAILURE;
  }
  if (const auto doc = json_utils::parse(payload.value(), "[payload]"); !doc)
  {
    shared_resources_->logger->publishFailureMessage(name(), doc.error());
    return BT::NodeStatus::FAILURE;
  }
  if (const auto sent = json_utils::sendDatagram(host.value(), port.value(), payload.value()); !sent)
  {
    shared_resources_->logger->publishFailureMessage(name(), sent.error());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
