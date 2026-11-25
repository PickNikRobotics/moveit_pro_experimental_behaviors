#pragma once

#include <string>
#include <unordered_map>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization_behaviors
{
/**
 * @brief Clears the stale triads/line a previous, longer contour left in a VisualizePath namespace.
 * @details A shorter VisualizePath publish leaves the higher triad ids of the previous one in place, and the UI
 * drops DELETEALL when "Auto-Clear on Run" is off. This sends a per-id DELETE for each stale marker instead.
 * See docs/AGENT_COMMENTS.md.
 *
 * It remembers the last waypoint count per marker_name, deletes the triads above the current count, and
 * deletes the line too when the path is empty. Run it just before the matching VisualizePath.
 */
class ClearStaleContourMarkers : public BT::SyncActionNode
{
public:
  ClearStaleContourMarkers(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /** @brief Ports: `path` (the contour about to be drawn), `marker_name`, and optional `topic`. */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  /** @brief Waypoint count last drawn under each marker_name, used to bound the stale-triad delete. */
  std::unordered_map<std::string, std::size_t> prev_pose_count_;
};
}  // namespace visualization_behaviors
