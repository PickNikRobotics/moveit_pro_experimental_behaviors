#include "visualization_behaviors/clear_stale_contour_markers.hpp"
#include <moveit_pro_behavior_interface/behavior_context.hpp>

#include "spdlog/spdlog.h"

namespace
{
constexpr auto kPortIDPath = "path";
constexpr auto kPortIDMarkerName = "marker_name";
constexpr auto kPortIDTopic = "topic";
constexpr auto kDefaultMarkerArrayTopic = "/visual_markers";

// Frame the markers were published in (VisualizePath transforms the contour to "world"). Ignored by
// the UI for DELETE (which matches on namespace + id), but set for correctness.
constexpr auto kWorldFrame = "world";

// VisualizePath publishes the polyline as a single LINE_LIST at this id; triads use ids 0..3N-1.
constexpr int kLineMarkerId = 1;
}  // namespace

namespace visualization_behaviors
{
ClearStaleContourMarkers::ClearStaleContourMarkers(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : BT::SyncActionNode(name, config), shared_resources_(shared_resources)
{
}

BT::PortsList ClearStaleContourMarkers::providedPorts()
{
  return { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
               kPortIDPath, "The contour about to be drawn by the matching VisualizePath; its size sets "
                            "how many triads will exist this pass."),
           BT::InputPort<std::string>(kPortIDMarkerName,
                                      "VisualizePath marker_name whose stale markers to clear (e.g. left_path)."),
           BT::InputPort<std::string>(kPortIDTopic, kDefaultMarkerArrayTopic, "Marker topic to publish DELETEs on.") };
}

BT::KeyValueVector ClearStaleContourMarkers::metadata()
{
  return { { "description", "Deletes the stale triads/line a longer previous contour left in a VisualizePath "
                            "namespace, using per-id DELETE (honored regardless of the UI's Auto-Clear on Run)." },
           { "subcategory", "Visualization" } };
}

BT::NodeStatus ClearStaleContourMarkers::tick()
{
  std::string marker_name;
  if (!getInput<std::string>(kPortIDMarkerName, marker_name) || marker_name.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing or empty input port [marker_name]");
    return BT::NodeStatus::FAILURE;
  }

  const auto path = getInput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPath);
  if (!path.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing required input port [path]: " + path.error());
    return BT::NodeStatus::FAILURE;
  }

  // VisualizePath draws nothing for a path shorter than two poses, so the effective triad count is 0
  // in that case (and we must also delete the line, which won't be republished).
  const std::size_t current_count = path.value().size() >= 2 ? path.value().size() : 0;
  const std::size_t prev_count = prev_pose_count_[marker_name];

  const std::string poses_ns = marker_name + " - Poses";
  const std::string line_ns = marker_name + " - Line";

  visualization_msgs::msg::MarkerArray array;
  auto push_delete = [&](const std::string& ns, int id) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = kWorldFrame;
    del.ns = ns;
    del.id = id;
    del.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(del);
  };

  // Delete triads above the count VisualizePath is about to (re)publish; it overwrites 0..3*current-1.
  for (std::size_t id = 3 * current_count; id < 3 * prev_count; ++id)
  {
    push_delete(poses_ns, static_cast<int>(id));
  }
  // An empty contour publishes no line this pass, so drop the previous one explicitly.
  if (current_count == 0)
  {
    push_delete(line_ns, kLineMarkerId);
  }

  if (!array.markers.empty())
  {
    const std::string topic = getInput<std::string>(kPortIDTopic).value_or(kDefaultMarkerArrayTopic);
    if (!marker_publisher_ || marker_publisher_->get_topic_name() != topic)
    {
      marker_publisher_ = shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
    }
    marker_publisher_->publish(array);
    spdlog::info("Cleared {} stale contour marker(s) for '{}'", array.markers.size(), marker_name);
  }

  prev_pose_count_[marker_name] = current_count;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace visualization_behaviors
