// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "test_http_server.hpp"

#include <gtest/gtest.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

using nlohmann::json;

namespace experimental_behaviors::test
{
namespace
{
/// Loads this package's Behavior plugin into a factory and runs small trees against one blackboard.
class SendHttpRequestTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("send_http_request_test");
    context_ = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_, false);
    loader_ = class_loader_.createUniqueInstance("experimental_behaviors::ExperimentalBehaviorsLoader");
    loader_->registerBehaviors(factory_, context_);
  }

  BT::Tree& createTree(const std::string& body)
  {
    const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">)" + body +
                            "</BehaviorTree></root>";
    tree_ = factory_.createTreeFromText(xml, blackboard_);
    return tree_;
  }

  BT::NodeStatus run(const std::string& body)
  {
    return createTree(body).tickWhileRunning();
  }

  /// Returns, and clears, the failure messages the Behaviors published.
  std::string errors()
  {
    return context_->logger->consumeErrorLogBuffer();
  }

  std::string output(const std::string& key)
  {
    return blackboard_->get<std::string>(key);
  }

  /// Checks the outputs of a run that got no HTTP response, and that error_message holds @p expected.
  void expectNoResponse(const std::string& expected)
  {
    EXPECT_EQ(blackboard_->get<int>("status_code"), 0);
    EXPECT_EQ(output("response_body"), "");
    EXPECT_EQ(output("response_headers"), "{}");
    EXPECT_NE(output("error_message").find(expected), std::string::npos) << output("error_message");
    const auto message = errors();
    EXPECT_NE(message.find(expected), std::string::npos) << message;
  }

  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader_{
    "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase"
  };
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  pluginlib::UniquePtr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_ = BT::Blackboard::create();
  BT::Tree tree_;
};

/// An https server (python3 with a self-signed certificate made by openssl) in a child process.
class TlsTestServer
{
public:
  TlsTestServer()
  {
    char dir_template[] = "/tmp/send_http_request_tls_XXXXXX";
    if (mkdtemp(dir_template) == nullptr)
    {
      return;
    }
    dir_ = dir_template;
    const std::string make_cert = "openssl req -x509 -newkey rsa:2048 -nodes -days 1 -subj /CN=localhost -keyout " +
                                  dir_ + "/key.pem -out " + dir_ + "/cert.pem > /dev/null 2>&1";
    if (std::system(make_cert.c_str()) != 0)
    {
      return;
    }
    std::ofstream(dir_ + "/server.py") << R"(
import http.server, ssl, sys
class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        body = b'{"tls": true}'
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
    def log_message(self, *args):
        pass
server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(sys.argv[1], sys.argv[2])
server.socket = context.wrap_socket(server.socket, server_side=True)
print(server.server_address[1], flush=True)
server.serve_forever()
)";
    int out[2];
    if (pipe(out) != 0)
    {
      return;
    }
    pid_ = fork();
    if (pid_ == 0)
    {
      dup2(out[1], STDOUT_FILENO);
      close(out[0]);
      close(out[1]);
      const std::string script = dir_ + "/server.py";
      const std::string cert = dir_ + "/cert.pem";
      const std::string key = dir_ + "/key.pem";
      execlp("python3", "python3", script.c_str(), cert.c_str(), key.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
    close(out[1]);
    // The server prints its port once it listens.
    pollfd ready{ out[0], POLLIN, 0 };
    char buffer[16] = { 0 };
    if (pid_ > 0 && poll(&ready, 1, 10000) == 1 && read(out[0], buffer, sizeof(buffer) - 1) > 0)
    {
      port_ = std::atoi(buffer);
    }
    close(out[0]);
  }

  ~TlsTestServer()
  {
    if (pid_ > 0)
    {
      kill(pid_, SIGTERM);
      waitpid(pid_, nullptr, 0);
    }
    if (!dir_.empty())
    {
      std::error_code ignored;
      std::filesystem::remove_all(dir_, ignored);
    }
  }

  int port() const
  {
    return port_;
  }

private:
  std::string dir_;
  pid_t pid_ = -1;
  int port_ = 0;
};
}  // namespace

TEST_F(SendHttpRequestTest, GetSendsQueryAndHeadersAndReadsResponse)
{
  TestHttpServer server(HttpReply{ 200,
                                   { { "Content-Type", "application/json" },
                                     { "X-Request-Id", "17" },
                                     { "Set-Cookie", "a=1" },
                                     { "Set-Cookie", "b=2" } },
                                   R"({"items": []})" });
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/items?fixed=1") +
                R"(" method="GET" timeout="5.0" verify_tls="true" follow_redirects="false"
                  query_parameters='{"name": "arm 1", "count": 2, "ready": true, "tag": ["a", "b"], "flag": null}'
                  headers='{"X-Api-Key": "secret", "Accept": "application/json", "X-Empty": null, "X-Number": 7}'
                  status_code="{code}" response_body="{body}" response_headers="{headers}" error_message="{error}"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(blackboard_->get<int>("code"), 200);
  EXPECT_EQ(json::parse(output("body")), json::parse(R"({"items": []})"));
  const auto headers = json::parse(output("headers"));
  EXPECT_EQ(headers["x-request-id"], "17");
  EXPECT_EQ(headers["set-cookie"], "a=1, b=2");
  EXPECT_EQ(headers["content-type"], "application/json");
  EXPECT_EQ(output("error"), "");

  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "GET");
  EXPECT_EQ(requests[0].target, "/items?fixed=1&name=arm%201&count=2&ready=true&tag=a&tag=b&flag");
  EXPECT_EQ(requests[0].headers.at("x-api-key"), "secret");
  EXPECT_EQ(requests[0].headers.at("accept"), "application/json");
  EXPECT_EQ(requests[0].headers.at("x-empty"), "");
  EXPECT_EQ(requests[0].headers.at("x-number"), "7");
  EXPECT_FALSE(requests[0].hasHeader("content-type"));
  EXPECT_EQ(requests[0].body, "");
}

TEST_F(SendHttpRequestTest, DefaultsAreGetAndDefaultOutputKeys)
{
  TestHttpServer server(HttpReply{ 200, {}, "plain text" });
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/") + R"("/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(blackboard_->get<int>("status_code"), 200);
  EXPECT_EQ(output("response_body"), "plain text");
  EXPECT_EQ(json::parse(output("response_headers"))["content-length"], "10");
  EXPECT_EQ(output("error_message"), "");
  ASSERT_EQ(server.requests().size(), 1u);
  EXPECT_EQ(server.requests()[0].method, "GET");
}

TEST_F(SendHttpRequestTest, PostSendsJsonBodyWithDefaultContentType)
{
  TestHttpServer server(HttpReply{ 201, {}, R"({"id": 42})" });
  // Large enough that curl would wait for "100 Continue" if the Behavior did not turn that off.
  const std::string padding(4096, 'x');
  blackboard_->set<std::string>("doc", json{ { "robot", "arm" }, { "padding", padding } }.dump());
  const auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/jobs") + R"(" method="POST" body="{doc}"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(900));
  EXPECT_EQ(blackboard_->get<int>("status_code"), 201);
  EXPECT_EQ(json::parse(output("response_body"))["id"], 42);

  // JSON typed straight into the port is sent as it is, not read as a blackboard key.
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/jobs") +
                R"(" method="post" body='{"robot": "arm", "state": "idle"}'/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();

  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(requests[0].method, "POST");
  EXPECT_EQ(requests[0].target, "/jobs");
  EXPECT_EQ(requests[0].headers.at("content-type"), "application/json");
  EXPECT_FALSE(requests[0].hasHeader("expect"));
  EXPECT_EQ(json::parse(requests[0].body)["padding"], padding);
  EXPECT_EQ(requests[1].method, "POST");
  EXPECT_EQ(requests[1].body, R"({"robot": "arm", "state": "idle"})");
}

TEST_F(SendHttpRequestTest, ReplacesSendJsonHttp)
{
  // The port mapping in docs/restful_behaviors.md: the same request SendJsonHttp sent, and its two outputs.
  TestHttpServer server(HttpReply{ 200, {}, R"({"accepted": true})" });
  blackboard_->set<std::string>("json", R"({"robot":"arm"})");
  const std::string send = R"(<SendHttpRequest url=")" + server.url("/status") +
                           R"(" method="POST" body="{json}" headers='{"Accept": "application/json"}' timeout="10.0"
                             response_body="{response}" status_code="{status_code}"/>)";
  ASSERT_EQ(run(send), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(json::parse(output("response")), json::parse(R"({"accepted": true})"));
  EXPECT_EQ(blackboard_->get<int>("status_code"), 200);
  auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "POST");
  EXPECT_EQ(requests[0].headers.at("content-type"), "application/json");
  EXPECT_EQ(requests[0].headers.at("accept"), "application/json");
  EXPECT_EQ(requests[0].body, R"({"robot":"arm"})");

  // SendJsonHttp refused invalid JSON before sending; CreateJson in front does the same.
  blackboard_->set<std::string>("json", "{not json");
  EXPECT_EQ(run(R"(<Sequence><CreateJson initial="{json}" json="{json}"/>)" + send + "</Sequence>"),
            BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("is not valid JSON"), std::string::npos) << message;
  EXPECT_EQ(server.requests().size(), 1u);
}

TEST_F(SendHttpRequestTest, PutSendsBlackboardBodyWithCustomContentType)
{
  TestHttpServer server(HttpReply{ 204, {}, "" });
  blackboard_->set<std::string>("text", "hello robot");
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/notes/1") +
                R"(" method="put" body="{text}" content_type="text/plain"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(blackboard_->get<int>("status_code"), 204);
  EXPECT_EQ(output("response_body"), "");
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "PUT");
  EXPECT_EQ(requests[0].headers.at("content-type"), "text/plain");
  EXPECT_EQ(requests[0].body, "hello robot");
}

TEST_F(SendHttpRequestTest, PatchUsesContentTypeFromHeaders)
{
  TestHttpServer server(HttpReply{ 200, {}, "{}" });
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/robots/arm") +
                R"(" method="PATCH" body='{"state": "busy"}'
                  headers='{"Content-Type": "application/merge-patch+json"}'/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "PATCH");
  // Exactly one Content-Type: the one from headers wins over content_type.
  EXPECT_EQ(requests[0].headers.at("content-type"), "application/merge-patch+json");
  EXPECT_EQ(requests[0].body, R"({"state": "busy"})");
}

TEST_F(SendHttpRequestTest, DeleteSendsBodyOnlyWhenGiven)
{
  TestHttpServer server(HttpReply{ 200, {}, "" });
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/jobs/7") + R"(" method="DELETE"/>)"), BT::NodeStatus::SUCCESS)
      << errors();
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/jobs") + R"(" method="DELETE" body='["7", "8"]'/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(requests[0].method, "DELETE");
  EXPECT_FALSE(requests[0].hasHeader("content-type"));
  EXPECT_EQ(requests[0].body, "");
  EXPECT_EQ(requests[1].method, "DELETE");
  EXPECT_EQ(requests[1].headers.at("content-type"), "application/json");
  EXPECT_EQ(requests[1].body, R"(["7", "8"])");
}

TEST_F(SendHttpRequestTest, PostWithEmptyBodyAndNoContentType)
{
  TestHttpServer server(HttpReply{ 202, {}, "" });
  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/trigger") + R"(" method="POST" content_type=""/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].method, "POST");
  EXPECT_EQ(requests[0].headers.at("content-length"), "0");
  EXPECT_FALSE(requests[0].hasHeader("content-type"));
}

TEST_F(SendHttpRequestTest, ErrorStatusFailsButWritesOutputs)
{
  TestHttpServer server(HttpReply{ 404, { { "X-Reason", "missing" } }, R"({"error": "no such job"})" });
  EXPECT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/jobs/99?token=secret") + R"("/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard_->get<int>("status_code"), 404);
  EXPECT_EQ(json::parse(output("response_body")), json::parse(R"({"error": "no such job"})"));
  EXPECT_EQ(json::parse(output("response_headers"))["x-reason"], "missing");
  const std::string error = output("error_message");
  EXPECT_NE(error.find("GET http://127.0.0.1"), std::string::npos) << error;
  EXPECT_NE(error.find("returned HTTP status 404"), std::string::npos) << error;
  // The query string, which may hold a token, stays out of the message.
  EXPECT_EQ(error.find("secret"), std::string::npos) << error;
  const auto message = errors();
  EXPECT_NE(message.find("returned HTTP status 404"), std::string::npos) << message;
}

TEST_F(SendHttpRequestTest, FollowRedirectsOnlyWhenAsked)
{
  TestHttpServer server([](const HttpRequest& request) {
    if (request.target == "/old")
    {
      return HttpReply{ 302, { { "Location", "/new" } }, "" };
    }
    return HttpReply{ 200, { { "X-Final", "yes" } }, "moved here" };
  });
  EXPECT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/old") + R"("/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard_->get<int>("status_code"), 302);
  EXPECT_EQ(json::parse(output("response_headers"))["location"], "/new");
  EXPECT_EQ(server.requests().size(), 1u);
  errors();

  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/old") + R"(" follow_redirects="true"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(blackboard_->get<int>("status_code"), 200);
  EXPECT_EQ(output("response_body"), "moved here");
  // Only the final response's headers are kept.
  const auto headers = json::parse(output("response_headers"));
  EXPECT_EQ(headers["x-final"], "yes");
  EXPECT_FALSE(headers.contains("location"));
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 3u);
  EXPECT_EQ(requests[2].target, "/new");
}

TEST_F(SendHttpRequestTest, RejectsBadInputs)
{
  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" method="FETCH"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[method] must be GET, POST, PUT, PATCH or DELETE");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" body="data"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[body] must be empty for a GET request");

  EXPECT_EQ(run(R"(<SendHttpRequest url="ftp://127.0.0.1/file"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[url] must start with http:// or https://");

  EXPECT_EQ(run(R"(<SendHttpRequest url="file:///etc/hostname"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[url] must start with http:// or https://");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" timeout="0"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[timeout] must be a finite number of seconds > 0");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" query_parameters="{bad"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[query_parameters] is not valid JSON");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" query_parameters="[1, 2]"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[query_parameters] must be a JSON object, not array");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" query_parameters='{"a": {"b": 1}}'/>)"),
            BT::NodeStatus::FAILURE);
  expectNoResponse("[query_parameters] value of 'a' is object");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" headers='{"Bad Name": "x"}'/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[headers] 'Bad Name' is not a valid header name");

  // A value with a line break could add a header of its own, so it is refused.
  blackboard_->set<std::string>("bad_headers", json{ { "X-A", "a\r\nX-Injected: b" } }.dump());
  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" headers="{bad_headers}"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[headers] value of 'X-A' contains a line break");

  blackboard_->set<std::string>("bad_type", "text/plain\r\nX-Injected: b");
  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" method="POST" content_type="{bad_type}"/>)"),
            BT::NodeStatus::FAILURE);
  expectNoResponse("[content_type] contains a line break");

  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:9/" headers='{"X-A": ["a"]}'/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("[headers] value of 'X-A' is array");

  EXPECT_EQ(run(R"(<SendHttpRequest method="GET"/>)"), BT::NodeStatus::FAILURE);
  expectNoResponse("Failed to get required input");
}

TEST_F(SendHttpRequestTest, UnreachableServerClearsOutputs)
{
  // Values left by an earlier run must not survive a run that got no response.
  blackboard_->set<int>("status_code", 200);
  blackboard_->set<std::string>("response_body", "stale");
  blackboard_->set<std::string>("response_headers", R"({"stale": "yes"})");
  const int closed_port = freeTcpPort();
  EXPECT_EQ(run(R"(<SendHttpRequest url="http://127.0.0.1:)" + std::to_string(closed_port) + R"(/"/>)"),
            BT::NodeStatus::FAILURE);
  expectNoResponse("failed");
}

TEST_F(SendHttpRequestTest, TimesOut)
{
  TestHttpServer server(HttpReply{ 200, {}, "{}", std::chrono::milliseconds(3000) });
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(run(R"(<SendHttpRequest url=")" + server.url("/") + R"(" timeout="0.3"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(2000));
  expectNoResponse("imed out");
}

TEST_F(SendHttpRequestTest, HaltCancelsRequestQuickly)
{
  TestHttpServer server(HttpReply{ 200, {}, "{}", std::chrono::milliseconds(5000) });
  auto& tree = createTree(R"(<SendHttpRequest url=")" + server.url("/") + R"(" timeout="30"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto start = std::chrono::steady_clock::now();
  tree.haltTree();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(500));
  EXPECT_EQ(blackboard_->get<int>("status_code"), 0);
  EXPECT_NE(output("error_message").find("cancelled because the Behavior was halted"), std::string::npos)
      << output("error_message");
}

TEST_F(SendHttpRequestTest, VerifyTlsChecksTheCertificate)
{
  TlsTestServer server;
  if (server.port() == 0)
  {
    GTEST_SKIP() << "python3 or openssl is not available to run an https test server.";
  }
  const std::string url = "https://127.0.0.1:" + std::to_string(server.port()) + "/";
  EXPECT_EQ(run(R"(<SendHttpRequest url=")" + url + R"("/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard_->get<int>("status_code"), 0);
  EXPECT_NE(output("error_message").find("certificate"), std::string::npos) << output("error_message");
  errors();

  ASSERT_EQ(run(R"(<SendHttpRequest url=")" + url + R"(" verify_tls="false"/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(blackboard_->get<int>("status_code"), 200);
  EXPECT_EQ(json::parse(output("response_body"))["tls"], true);
}
}  // namespace experimental_behaviors::test

int main(int argc, char** argv)
{
  // Keep a proxy set in the environment away from the loopback test servers.
  setenv("NO_PROXY", "*", 1);
  setenv("no_proxy", "*", 1);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
