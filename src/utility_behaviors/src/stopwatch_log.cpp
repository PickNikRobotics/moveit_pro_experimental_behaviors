#include <behaviortree_cpp/basic_types.h>

#include <chrono>
#include <iomanip>
#include <moveit_pro_behavior_interface/clock_interface.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sstream>
#include "utility_behaviors/stopwatch_log.hpp"

namespace
{
inline constexpr auto kDescriptionStopwatchLog = R"(
                <p>
                    Log a message and the time elapsed since a timepoint provided on an input data port.
                </p>
            )";
constexpr auto kPortIDTimepoint = "timepoint";
constexpr auto kPortIDMessage = "message";
constexpr auto kPortIDLogMessageBool = "log_message_bool";
constexpr auto kPortIDElapsedTimeDouble_s = "elapsed_time_double_s";
constexpr auto kPortIDElapsedTimeString = "elapsed_time_str";
}  // namespace

namespace utility_behaviors
{
StopwatchLog::StopwatchLog(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
  , clock_interface_{ std::make_unique<moveit_pro::behaviors::SteadyClockInterface>() }
{
}

StopwatchLog::StopwatchLog(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
                           std::unique_ptr<moveit_pro::behaviors::ClockInterfaceBase> clock_interface)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
  , clock_interface_{ std::move(clock_interface) }
{
}

BT::PortsList StopwatchLog::providedPorts()
{
  return { BT::InputPort<std::chrono::time_point<std::chrono::steady_clock>>(
               kPortIDTimepoint, "{timepoint}", "Measure time elapsed since this timepoint."),
           BT::InputPort<std::string>(kPortIDMessage, "Message to log (required if log_message_bool is true)."),
           BT::OutputPort<std::string>(kPortIDElapsedTimeString, "{elapsed_time_str}",
                                       "Elapsed time (seconds) as a formatted string"),
           BT::OutputPort<double>(kPortIDElapsedTimeDouble_s, "{elapsed_time_double_s}",
                                  "Elapsed time double (seconds)"),
           BT::InputPort<bool>(kPortIDLogMessageBool, "true", "Whether to log the message") };
}

BT::KeyValueVector StopwatchLog::metadata()
{
  return { { "subcategory", "Utility" }, { "description", kDescriptionStopwatchLog } };
}

BT::NodeStatus StopwatchLog::tick()
{
  const auto now = clock_interface_->getTimePoint();

  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::chrono::time_point<std::chrono::steady_clock>>(kPortIDTimepoint),
      getInput<bool>(kPortIDLogMessageBool));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [before, log_message_bool] = ports.value();

  auto message_in = getInput<std::string>(kPortIDMessage);
  // Enforce requirement of the message input only if log_message_bool is true
  if (log_message_bool && !message_in)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Missing required input [message] when log_message_bool is true: " + message_in.error());
    return BT::NodeStatus::FAILURE;
  }

  const std::chrono::duration<double> delta{ now - before };
  std::stringstream out;
  out << std::fixed << std::setprecision(3) << delta.count();
  const std::string elapsed_str = out.str();

  if (log_message_bool)
  {
    const std::string message = message_in.value_or("");
    getBehaviorContext()->logger->publishInfoMessage(name(),
                                                     "Time elapsed: " + elapsed_str + "s" + ", message: " + message);
  }

  // Set output ports
  setOutput(kPortIDElapsedTimeString, elapsed_str);

  const double elapsed_sec = delta.count();
  setOutput(kPortIDElapsedTimeDouble_s, elapsed_sec);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace utility_behaviors
