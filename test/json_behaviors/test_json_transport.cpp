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

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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

/// One request as the test server saw it.
struct Request
{
  std::string method;
  std::string target;
  std::string content_type;
  std::string body;
};

/// Minimal HTTP/1.1 server on the loopback interface: answers every request with a fixed reply.
class TestHttpServer
{
public:
  TestHttpServer(int status, std::string reply, std::chrono::milliseconds delay = std::chrono::milliseconds(0))
    : status_(status), reply_(std::move(reply)), delay_(delay)
  {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    const int enable = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in addr = loopback(0);
    bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t length = sizeof(addr);
    getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &length);
    port_ = ntohs(addr.sin_port);
    listen(listen_fd_, 4);
    thread_ = std::thread([this] { serve(); });
  }

  ~TestHttpServer()
  {
    stop_ = true;
    shutdown(listen_fd_, SHUT_RDWR);
    close(listen_fd_);
    thread_.join();
  }

  std::string url(const std::string& path) const
  {
    return "http://127.0.0.1:" + std::to_string(port_) + path;
  }

  std::vector<Request> requests()
  {
    std::scoped_lock lock(mutex_);
    return requests_;
  }

private:
  void serve()
  {
    while (!stop_)
    {
      const int fd = accept(listen_fd_, nullptr, nullptr);
      if (fd < 0)
      {
        return;
      }
      handle(fd);
      close(fd);
    }
  }

  void handle(int fd)
  {
    std::string data;
    char buffer[4096];
    std::size_t header_end = std::string::npos;
    while ((header_end = data.find("\r\n\r\n")) == std::string::npos)
    {
      const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0)
      {
        return;
      }
      data.append(buffer, static_cast<std::size_t>(n));
    }
    const std::string head = data.substr(0, header_end);
    Request request;
    request.method = head.substr(0, head.find(' '));
    const auto target_start = head.find(' ') + 1;
    request.target = head.substr(target_start, head.find(' ', target_start) - target_start);
    std::size_t content_length = 0;
    std::size_t line_start = head.find("\r\n");
    while (line_start != std::string::npos)
    {
      const std::size_t next = head.find("\r\n", line_start + 2);
      std::string line =
          head.substr(line_start + 2, next == std::string::npos ? std::string::npos : next - line_start - 2);
      for (auto& c : line)
      {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      if (line.rfind("content-length:", 0) == 0)
      {
        content_length = std::stoul(line.substr(15));
      }
      if (line.rfind("content-type:", 0) == 0)
      {
        request.content_type = line.substr(line.find_first_not_of(' ', 13));
      }
      line_start = next;
    }
    request.body = data.substr(header_end + 4);
    while (request.body.size() < content_length)
    {
      const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0)
      {
        return;
      }
      request.body.append(buffer, static_cast<std::size_t>(n));
    }
    {
      std::scoped_lock lock(mutex_);
      requests_.push_back(request);
    }
    const auto wake = std::chrono::steady_clock::now() + delay_;
    while (!stop_ && std::chrono::steady_clock::now() < wake)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const std::string response =
        "HTTP/1.1 " + std::to_string(status_) +
        " Test\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(reply_.size()) +
        "\r\nConnection: close\r\n\r\n" + reply_;
    send(fd, response.data(), response.size(), MSG_NOSIGNAL);
  }

  int status_;
  std::string reply_;
  std::chrono::milliseconds delay_;
  int listen_fd_ = -1;
  int port_ = 0;
  std::atomic<bool> stop_{ false };
  std::mutex mutex_;
  std::vector<Request> requests_;
  std::thread thread_;
};
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

TEST_F(JsonTransportTest, HttpPostSendsJsonAndReadsResponse)
{
  TestHttpServer server(200, R"({"accepted": true})");
  // Large enough that curl would wait for "100 Continue" if the Behavior did not turn that off.
  const std::string padding(4096, 'x');
  blackboard_->set<std::string>("doc", json{ { "robot", "arm" }, { "padding", padding } }.dump());
  const auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(run(R"(<SendJsonHttp url=")" + server.url("/status") +
                R"(" payload="{doc}" response="{response}" status_code="{code}"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(900));
  EXPECT_EQ(json::parse(blackboard_->get<std::string>("response")), json::parse(R"({"accepted": true})"));
  EXPECT_EQ(blackboard_->get<int>("code"), 200);

  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "POST");
  EXPECT_EQ(requests[0].target, "/status");
  EXPECT_EQ(requests[0].content_type, "application/json");
  EXPECT_EQ(json::parse(requests[0].body)["padding"], padding);
}

TEST_F(JsonTransportTest, HttpPutWithLiteralPayload)
{
  TestHttpServer server(204, "");
  ASSERT_EQ(run(R"(<SendJsonHttp url=")" + server.url("/robots/arm") +
                R"(" method="put" payload='{"state": "idle"}' response="{response}" status_code="{code}"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(blackboard_->get<int>("code"), 204);
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "PUT");
  EXPECT_EQ(json::parse(requests[0].body), json::parse(R"({"state": "idle"})"));
}

TEST_F(JsonTransportTest, HttpErrorStatusFailsButSetsOutputs)
{
  TestHttpServer server(503, R"({"error": "busy"})");
  EXPECT_EQ(run(R"(<SendJsonHttp url=")" + server.url("/status") +
                R"(" payload="[]" response="{response}" status_code="{code}"/>)"),
            BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("returned HTTP status 503"), std::string::npos) << message;
  EXPECT_EQ(blackboard_->get<int>("code"), 503);
  EXPECT_EQ(json::parse(blackboard_->get<std::string>("response")), json::parse(R"({"error": "busy"})"));
}

TEST_F(JsonTransportTest, HttpRejectsBadInputsAndUnreachableServer)
{
  EXPECT_EQ(run(R"(<SendJsonHttp url="http://127.0.0.1:9/" method="GET" payload="[]"/>)"), BT::NodeStatus::FAILURE);
  auto message = errors();
  EXPECT_NE(message.find("must be POST or PUT"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<SendJsonHttp url="http://127.0.0.1:9/" payload="{bad"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("[payload] is not valid JSON"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<SendJsonHttp url="file:///etc/hostname" payload="[]"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("failed"), std::string::npos) << message;

  const int closed_port = freePort(SOCK_STREAM);
  EXPECT_EQ(run(R"(<SendJsonHttp url="http://127.0.0.1:)" + std::to_string(closed_port) + R"(/" payload="[]"/>)"),
            BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("failed"), std::string::npos) << message;
}

TEST_F(JsonTransportTest, HttpTimesOut)
{
  TestHttpServer server(200, "{}", std::chrono::milliseconds(3000));
  EXPECT_EQ(run(R"(<SendJsonHttp url=")" + server.url("/") + R"(" payload="[]" timeout="0.3"/>)"),
            BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("failed"), std::string::npos) << message;
}

TEST_F(JsonTransportTest, HttpHaltCancelsRequest)
{
  TestHttpServer server(200, "{}", std::chrono::milliseconds(5000));
  auto& tree = createTree(R"(<SendJsonHttp url=")" + server.url("/") + R"(" payload="[]" timeout="30"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto start = std::chrono::steady_clock::now();
  tree.haltTree();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(2000));
}
}  // namespace experimental_behaviors::test

int main(int argc, char** argv)
{
  // Keep a proxy set in the environment away from the loopback test server.
  setenv("NO_PROXY", "*", 1);
  setenv("no_proxy", "*", 1);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
