// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/receive_json_udp.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <cmath>

namespace
{
inline constexpr auto kDescriptionReceiveJsonUdp = R"(
                <p>
                    Waits for a JSON document sent by another system as a UDP datagram, and outputs it as
                    JSON text for <code>GetJsonField</code> and the other JSON Behaviors.
                </p>
                <p>
                    The socket opens on the first tick and stays open until the Objective ends, so datagrams
                    that arrive between runs are kept. Each run outputs the newest datagram and drops older
                    ones, which suits status or command streams where only the latest value matters. If no
                    datagram is waiting, the Behavior keeps running until one arrives, and fails after
                    <code>timeout</code> seconds (0 checks once without waiting). A datagram that is not
                    valid JSON fails the Behavior.
                </p>
                <p>
                    Security: UDP is not an authenticated channel; any process that can reach the port can
                    send to it. <code>bind_address</code> defaults to the loopback interface
                    (<code>127.0.0.1</code>), which accepts datagrams from this computer only. Set it to the
                    address of one network interface to accept datagrams from that network, and use
                    <code>0.0.0.0</code> (all interfaces) only on a trusted network. Validate every field you
                    read before you act on it.
                </p>
            )";

constexpr auto kPortIDPort = "port";
constexpr auto kPortIDBindAddress = "bind_address";
constexpr auto kPortIDTimeout = "timeout";
constexpr auto kPortIDPayload = "payload";
}  // namespace

namespace experimental_behaviors
{
ReceiveJsonUdp::ReceiveJsonUdp(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList ReceiveJsonUdp::providedPorts()
{
  return {
    BT::InputPort<int>(kPortIDPort, "UDP port to listen on."),
    BT::InputPort<std::string>(kPortIDBindAddress, "127.0.0.1",
                               "Local address to listen on. The default accepts datagrams from this computer only; "
                               "0.0.0.0 accepts them on every interface."),
    BT::InputPort<double>(kPortIDTimeout, 1.0, "Seconds to wait for a datagram before failing. 0 does not wait."),
    BT::OutputPort<std::string>(kPortIDPayload, "{payload}", "The newest datagram received, as JSON text."),
  };
}

BT::KeyValueVector ReceiveJsonUdp::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionReceiveJsonUdp } };
}

BT::NodeStatus ReceiveJsonUdp::onStart()
{
  const auto port = getInput<int>(kPortIDPort);
  const auto bind_address = getInput<std::string>(kPortIDBindAddress);
  const auto timeout = getInput<double>(kPortIDTimeout);
  if (!port || !bind_address || !timeout)
  {
    const std::string error = !port ? port.error() : (!bind_address ? bind_address.error() : timeout.error());
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " + error);
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(timeout.value()) || timeout.value() < 0.0)
  {
    shared_resources_->logger->publishFailureMessage(name(), "[timeout] must be a finite number of seconds >= 0.");
    return BT::NodeStatus::FAILURE;
  }

  // Keep the socket across runs; only a new address or port opens a new one.
  if (!socket_.isOpen() || bind_address.value() != bound_address_ || port.value() != bound_port_)
  {
    socket_ = json_utils::UdpSocket();
    auto socket = json_utils::UdpSocket::bind(bind_address.value(), port.value());
    if (!socket)
    {
      shared_resources_->logger->publishFailureMessage(name(), socket.error());
      return BT::NodeStatus::FAILURE;
    }
    socket_ = std::move(socket.value());
    bound_address_ = bind_address.value();
    bound_port_ = port.value();
  }

  timeout_ = timeout.value();
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_));
  const auto status = pollSocket();
  if (status == BT::NodeStatus::RUNNING && timeout_ == 0.0)
  {
    shared_resources_->logger->publishFailureMessage(name(), "No datagram waiting on " + bound_address_ + ":" +
                                                                 std::to_string(bound_port_) + ".");
    return BT::NodeStatus::FAILURE;
  }
  return status;
}

BT::NodeStatus ReceiveJsonUdp::onRunning()
{
  const auto status = pollSocket();
  if (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() >= deadline_)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Timed out after " + std::to_string(timeout_) +
                                                                 " s waiting for a datagram on " + bound_address_ +
                                                                 ":" + std::to_string(bound_port_) + ".");
    return BT::NodeStatus::FAILURE;
  }
  return status;
}

void ReceiveJsonUdp::onHalted()
{
  // The socket stays open so datagrams that arrive before the next run are kept.
}

BT::NodeStatus ReceiveJsonUdp::pollSocket()
{
  const auto datagram = socket_.receiveLatest();
  if (!datagram)
  {
    shared_resources_->logger->publishFailureMessage(name(), datagram.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!datagram->has_value())
  {
    return BT::NodeStatus::RUNNING;
  }
  const std::string& payload = datagram->value();
  if (const auto doc = json_utils::parse(payload, "The received datagram"); !doc)
  {
    shared_resources_->logger->publishFailureMessage(name(), doc.error());
    return BT::NodeStatus::FAILURE;
  }
  setOutput(kPortIDPayload, payload);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
