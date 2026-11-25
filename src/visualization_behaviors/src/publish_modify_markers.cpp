#include "visualization_behaviors/publish_modify_markers.hpp"
#include <moveit_pro_behavior_interface/behavior_context.hpp>

#include "spdlog/spdlog.h"

namespace visualization_behaviors
{
PublishModifyMarkers::PublishModifyMarkers(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : BT::SyncActionNode(name, config), shared_resources_(shared_resources)
{
}

BT::PortsList PublishModifyMarkers::providedPorts()
{
  return { BT::InputPort<std::string>("namespace", "Namespace for the markers (used to modify/delete markers later)"),
           BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("marker_poses",
                                                                       "Vector of poses for the markers"),
           BT::InputPort<std::vector<double>>("marker_scale", "Scale of the markers in xyz [x, y, z]"),
           BT::InputPort<std_msgs::msg::ColorRGBA>("color", "Color of the markers in RGBA"),
           BT::InputPort<int>("marker_type", "Marker type (0=ARROW, 1=CUBE, 2=SPHERE, 3=CYLINDER, etc.)"),
           BT::InputPort<int>("operation_type", 0, "Marker action value copied into each marker") };
}

BT::KeyValueVector PublishModifyMarkers::metadata()
{
  return { { "description", "Publishes or modifies visualization markers on the /visual_markers topic" },
           { "subcategory", "Visualization" } };
}

BT::NodeStatus PublishModifyMarkers::tick()
{
  // Create publisher if it doesn't exist
  if (!marker_publisher_)
  {
    auto node = shared_resources_->node;
    marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/visual_markers", 10);
    spdlog::info("Created publisher for /visual_markers topic");
  }

  // Get mandatory namespace port
  auto namespace_result = getInput<std::string>("namespace");
  if (!namespace_result.has_value() || namespace_result.value().empty())
  {
    spdlog::error("'namespace' input port is required and cannot be empty");
    return BT::NodeStatus::FAILURE;
  }
  std::string marker_namespace = namespace_result.value();

  // Get operation type
  auto operation_type_result = getInput<int>("operation_type");
  const int operation_type = operation_type_result.value();

  // Get poses vector
  auto poses_result = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("marker_poses");
  if (!poses_result.has_value())
  {
    spdlog::error("'marker_poses' input port is required");
    return BT::NodeStatus::FAILURE;
  }
  const auto& poses = poses_result.value();

  if (poses.empty())
  {
    spdlog::warn("Empty poses vector provided. No markers will be published.");
    return BT::NodeStatus::SUCCESS;
  }

  // Get scale
  auto scale_result = getInput<std::vector<double>>("marker_scale");
  if (!scale_result.has_value())
  {
    spdlog::error("'marker_scale' input port is required");
    return BT::NodeStatus::FAILURE;
  }
  const auto& scale_vec = scale_result.value();

  // Validate scale vector has 3 elements
  if (scale_vec.size() != 3)
  {
    spdlog::error("'marker_scale' must contain exactly 3 elements [x, y, z], got {}", scale_vec.size());
    return BT::NodeStatus::FAILURE;
  }

  // Create scale message from vector
  geometry_msgs::msg::Vector3 scale;
  scale.x = scale_vec[0];
  scale.y = scale_vec[1];
  scale.z = scale_vec[2];

  // Get color
  auto color_result = getInput<std_msgs::msg::ColorRGBA>("color");
  if (!color_result.has_value())
  {
    spdlog::error("'color' input port is required");
    return BT::NodeStatus::FAILURE;
  }
  const auto& color = color_result.value();

  // Get marker type
  auto marker_type_result = getInput<int>("marker_type");
  if (!marker_type_result.has_value())
  {
    spdlog::error("'marker_type' input port is required");
    return BT::NodeStatus::FAILURE;
  }
  int marker_type = marker_type_result.value();

  // Create marker array
  visualization_msgs::msg::MarkerArray marker_array;

  // Create markers for each pose
  for (size_t i = 0; i < poses.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = poses[i].header;  // Use frame_id and timestamp from PoseStamped
    marker.ns = marker_namespace;
    marker.id = static_cast<int>(i);
    marker.type = marker_type;
    marker.action = operation_type;
    marker.pose = poses[i].pose;  // Extract pose from PoseStamped
    marker.scale = scale;
    marker.color = color;
    marker.lifetime = rclcpp::Duration(0, 0);  // Lifetime of 0 means the marker persists until deleted

    marker_array.markers.push_back(marker);
  }

  // Publish the marker array
  marker_publisher_->publish(marker_array);

  spdlog::info("Published {} markers with namespace '{}' to /visual_markers", poses.size(), marker_namespace);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace visualization_behaviors
