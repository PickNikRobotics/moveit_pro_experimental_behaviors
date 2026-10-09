// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "json_behavior_test_fixture.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <behaviortree_cpp/contrib/json.hpp>

#include <chrono>
#include <string>
#include <thread>

using nlohmann::json;

namespace experimental_behaviors::test
{
namespace
{
sockaddr_in loopback(int port)
{
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  return addr;
}

/// Asks the kernel for a port that is free right now.
int freePort(int type)
{
  const int fd = socket(AF_INET, type, 0);
  sockaddr_in addr = loopback(0);
  bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  socklen_t length = sizeof(addr);
  getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &length);
  close(fd);
  return ntohs(addr.sin_port);
}

void sendUdp(int port, const std::string& payload)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  const sockaddr_in addr = loopback(port);
  sendto(fd, payload.data(), payload.size(), 0, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
  close(fd);
}
}  // namespace

using JsonTransportTest = JsonBehaviorTest;

TEST_F(JsonTransportTest, UdpSendAndReceiveInOneTree)
{
  const int port = freePort(SOCK_DGRAM);
  // Parallel ticks the receiver first, so its socket is open before the sender runs.
  const std::string tree = R"(
      <Parallel success_count="2" failure_count="1">
        <ReceiveJsonUdp port=")" +
                           std::to_string(port) +
                           R"(" timeout="2.0" payload="{received}"/>
        <SendJsonUdp host="127.0.0.1" port=")" +
                           std::to_string(port) + R"(" payload='{"robot": "arm", "state": "idle"}'/>
      </Parallel>)";
  ASSERT_EQ(run(tree), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(json::parse(blackboard_->get<std::string>("received")), json::parse(R"({"robot": "arm", "state": "idle"})"));
}

TEST_F(JsonTransportTest, UdpReceiveKeepsSocketAndReturnsLatest)
{
  const int port = freePort(SOCK_DGRAM);
  auto& tree =
      createTree(R"(<ReceiveJsonUdp port=")" + std::to_string(port) + R"(" timeout="2.0" payload="{received}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING) << errors();
  sendUdp(port, R"({"seq": 1})");
  sendUdp(port, R"({"seq": 2})");
  sendUdp(port, R"({"seq": 3})");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(json::parse(blackboard_->get<std::string>("received"))["seq"], 3);

  // Datagrams sent while the tree is idle wait in the open socket for the next run.
  sendUdp(port, R"({"seq": 4})");
  sendUdp(port, R"({"seq": 5})");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(json::parse(blackboard_->get<std::string>("received"))["seq"], 5);
}

TEST_F(JsonTransportTest, UdpReceiveTimesOut)
{
  const int port = freePort(SOCK_DGRAM);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(run(R"(<ReceiveJsonUdp port=")" + std::to_string(port) + R"(" timeout="0.2" payload="{received}"/>)"),
            BT::NodeStatus::FAILURE);
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(200));
  auto message = errors();
  EXPECT_NE(message.find("Timed out"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<ReceiveJsonUdp port=")" + std::to_string(port) + R"(" timeout="0" payload="{received}"/>)"),
            BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("No datagram waiting"), std::string::npos) << message;
}

TEST_F(JsonTransportTest, UdpReceiveRejectsInvalidJson)
{
  const int port = freePort(SOCK_DGRAM);
  auto& tree =
      createTree(R"(<ReceiveJsonUdp port=")" + std::to_string(port) + R"(" timeout="2.0" payload="{received}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  sendUdp(port, "not json");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("The received datagram is not valid JSON"), std::string::npos) << message;
}

TEST_F(JsonTransportTest, UdpSendRejectsBadInputs)
{
  EXPECT_EQ(run(R"(<SendJsonUdp host="127.0.0.1" port="9" payload="{bad"/>)"), BT::NodeStatus::FAILURE);
  auto message = errors();
  EXPECT_NE(message.find("[payload] is not valid JSON"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<SendJsonUdp host="127.0.0.1" port="0" payload="[]"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("outside 1-65535"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<SendJsonUdp host="no-such-host.invalid" port="9" payload="[]"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("cannot resolve"), std::string::npos) << message;
}
}  // namespace experimental_behaviors::test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
