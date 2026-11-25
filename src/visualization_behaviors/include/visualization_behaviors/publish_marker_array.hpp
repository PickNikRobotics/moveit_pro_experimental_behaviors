#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization_behaviors
{
/**
 * @brief Publishes a pre-built visualization_msgs::msg::MarkerArray to /visual_markers.
 * @details Unlike PublishModifyMarkers, which constructs markers from pose/scale/color ports, this
 * behavior publishes a MarkerArray that already exists on the blackboard verbatim. It is used to
 * republish cached marker arrays (e.g. the per-pass contour-planning slabs produced by
 * GenerateWashPaths) at execution time so the visualization tracks the pass being run.
 */
class PublishMarkerArray : public BT::SyncActionNode
{
public:
  PublishMarkerArray(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Ports: a required `marker_array` input and an optional `topic` (defaults to
   * /visual_markers).
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /** @brief Reads the `marker_array` port and publishes it to the configured topic. */
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};
}  // namespace visualization_behaviors
