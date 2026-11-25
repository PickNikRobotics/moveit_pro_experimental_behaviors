#include "utility_behaviors/stopwatch_check.hpp"
#include <spdlog/spdlog.h>

namespace utility_behaviors
{

StopWatchCheck::StopWatchCheck(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList StopWatchCheck::providedPorts()
{
  return { BT::InputPort<std::chrono::time_point<std::chrono::steady_clock>>("timestamp",
                                                                             "The epoch timestamp to compare against"),
           BT::InputPort<int>("duration_msec", "The duration in milliseconds to check if elapsed") };
}

BT::KeyValueVector StopWatchCheck::metadata()
{
  return { { "description", "Checks if a specified duration has elapsed since a given timestamp" },
           { "subcategory", "Utility" } };
}

BT::NodeStatus StopWatchCheck::tick()
{
  // Get the timestamp from input port
  const auto timestamp_result = getInput<std::chrono::time_point<std::chrono::steady_clock>>("timestamp");
  if (!timestamp_result)
  {
    spdlog::error("StopWatchCheck: Failed to get 'timestamp' input");
    return BT::NodeStatus::FAILURE;
  }
  const auto timestamp = timestamp_result.value();

  // Get the duration in milliseconds from input port
  const auto duration_msec_result = getInput<int>("duration_msec");
  if (!duration_msec_result)
  {
    spdlog::error("StopWatchCheck: Failed to get 'duration_msec' input");
    return BT::NodeStatus::FAILURE;
  }
  const auto duration_msec = duration_msec_result.value();

  // Get current time
  const auto current_time = std::chrono::steady_clock::now();

  // Calculate elapsed time in milliseconds
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - timestamp);
  const auto elapsed_msec = elapsed.count();

  spdlog::info("StopWatchCheck: Elapsed time: {} ms, Required duration: {} ms", elapsed_msec, duration_msec);

  // Check if the elapsed time is greater than or equal to the specified duration
  if (elapsed_msec >= duration_msec)
  {
    spdlog::info("StopWatchCheck: Duration has elapsed - returning SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    spdlog::info("StopWatchCheck: Duration has NOT elapsed - returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace utility_behaviors
