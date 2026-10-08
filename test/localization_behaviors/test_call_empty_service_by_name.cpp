// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/localization_behaviors/call_empty_service_by_name.hpp>

#include "localization_test_helpers.hpp"

#include <atomic>

namespace
{
using experimental_behaviors::CallEmptyServiceByName;
using experimental_behaviors::Empty;
using experimental_behaviors::test::makeConfig;
using experimental_behaviors::test::SpunContext;
using experimental_behaviors::test::tickUntilDone;

// Stands in for the ROS client, so each test sets what the server does.
class FakeClient : public moveit_pro::behaviors::ClientInterfaceBase<Empty>
{
public:
  struct State
  {
    std::string service_name;
    bool server_available = true;
    bool respond = true;
  };
  explicit FakeClient(std::shared_ptr<State> state) : state_{ std::move(state) }
  {
  }
  void initialize(const std::string& service_name, std::chrono::duration<double> /*wait*/,
                  std::chrono::duration<double> /*result*/) override
  {
    state_->service_name = service_name;
  }
  bool waitForServiceServer() const override
  {
    return state_->server_available;
  }
  tl::expected<Empty::Response, std::string> syncSendRequest(const Empty::Request& /*request*/) override
  {
    if (!state_->respond)
    {
      return tl::make_unexpected("timed out");
    }
    return Empty::Response{};
  }
  void cancelRequest() override
  {
  }

private:
  std::shared_ptr<State> state_;
};

class CallEmptyServiceByNameTest : public ::testing::Test
{
protected:
  std::unique_ptr<CallEmptyServiceByName> make(bool set_service_name = true)
  {
    auto config = makeConfig();
    if (set_service_name)
    {
      config.input_ports["service_name"] = "/some_service";
    }
    config.input_ports["response_timeout"] = "1.0";
    config.input_ports["wait_for_server_available_timeout"] = "1.0";
    return std::make_unique<CallEmptyServiceByName>("CallEmptyServiceByName", config, spun.context,
                                                    std::make_unique<FakeClient>(state));
  }
  SpunContext spun;
  std::shared_ptr<FakeClient::State> state = std::make_shared<FakeClient::State>();
};
}  // namespace

TEST_F(CallEmptyServiceByNameTest, SucceedsOnAnyResponse)
{
  auto behavior = make();
  EXPECT_EQ(tickUntilDone(*behavior), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(state->service_name, "/some_service");
}

TEST_F(CallEmptyServiceByNameTest, FailsWhenNoServer)
{
  state->server_available = false;
  auto behavior = make();
  EXPECT_EQ(tickUntilDone(*behavior), BT::NodeStatus::FAILURE);
}

TEST_F(CallEmptyServiceByNameTest, FailsWhenNoResponse)
{
  state->respond = false;
  auto behavior = make();
  EXPECT_EQ(tickUntilDone(*behavior), BT::NodeStatus::FAILURE);
}

TEST_F(CallEmptyServiceByNameTest, FailsWithoutAServiceName)
{
  auto behavior = make(false);
  EXPECT_EQ(tickUntilDone(*behavior), BT::NodeStatus::FAILURE);
}

TEST_F(CallEmptyServiceByNameTest, FailsOnANonNumericTimeout)
{
  auto config = makeConfig();
  config.input_ports["service_name"] = "/some_service";
  config.input_ports["response_timeout"] = "soon";
  config.input_ports["wait_for_server_available_timeout"] = "1.0";
  CallEmptyServiceByName behavior{ "CallEmptyServiceByName", config, spun.context, std::make_unique<FakeClient>(state) };
  EXPECT_EQ(tickUntilDone(behavior), BT::NodeStatus::FAILURE);
}

// Through the real ROS client, against a real Empty server.
TEST(CallEmptyServiceByNameRos, CallsARealServer)
{
  SpunContext spun;
  std::atomic<int> calls{ 0 };
  const auto server = spun.node->create_service<Empty>("/localization_behaviors_test/empty",
                                                       [&calls](const std::shared_ptr<Empty::Request>,
                                                                std::shared_ptr<Empty::Response>) { ++calls; });
  auto config = makeConfig();
  config.input_ports["service_name"] = "/localization_behaviors_test/empty";
  config.input_ports["response_timeout"] = "3.0";
  config.input_ports["wait_for_server_available_timeout"] = "3.0";
  CallEmptyServiceByName behavior{ "CallEmptyServiceByName", config, spun.context };
  EXPECT_EQ(tickUntilDone(behavior), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, 1);
}

TEST(CallEmptyServiceByNameRos, FailsWhenNoRealServerExists)
{
  SpunContext spun;
  auto config = makeConfig();
  config.input_ports["service_name"] = "/localization_behaviors_test/missing_empty";
  config.input_ports["response_timeout"] = "0.5";
  config.input_ports["wait_for_server_available_timeout"] = "0.5";
  CallEmptyServiceByName behavior{ "CallEmptyServiceByName", config, spun.context };
  EXPECT_EQ(tickUntilDone(behavior), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
