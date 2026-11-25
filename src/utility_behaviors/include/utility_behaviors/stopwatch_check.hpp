#pragma once

#include <behaviortree_cpp/action_node.h>
#include <chrono>

namespace utility_behaviors
{

/**
 * @brief Checks if a specified duration has elapsed since a given timestamp.
 *
 * This behavior compares the current time with a provided timestamp and checks if the elapsed time
 * is greater than or equal to a specified duration in milliseconds.
 *
 * Data Port Name    | Port Type | Object Type
 * ----------------- | --------- | ----------------------------------------------------
 * timestamp         | input     | std::chrono::time_point<std::chrono::steady_clock>
 * duration_msec     | input     | int
 *
 * @return SUCCESS if the elapsed time since timestamp is greater than or equal to duration_msec.
 * @return FAILURE if the elapsed time since timestamp is less than duration_msec.
 */
class StopWatchCheck : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the StopWatchCheck behavior.
   * @param name The name of a particular instance of this Behavior.
   * @param config This contains runtime configuration info for this Behavior.
   */
  StopWatchCheck(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief Implementation of the required providedPorts() function for the StopWatchCheck Behavior.
   * @return A list of input/output ports for this behavior.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata in MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for StopWatchCheck.
   * @return SUCCESS if duration has elapsed, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;
};

}  // namespace utility_behaviors
