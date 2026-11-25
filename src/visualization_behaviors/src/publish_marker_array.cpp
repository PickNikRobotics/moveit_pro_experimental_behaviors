#include "visualization_behaviors/publish_marker_array.hpp"
#include <moveit_pro_behavior_interface/behavior_context.hpp>

#include "spdlog/spdlog.h"

namespace
{
constexpr auto kPortIDMarkerArray = "marker_array";
constexpr auto kPortIDTopic = "topic";
constexpr auto kDefaultMarkerArrayTopic = "/visual_markers";
}  // namespace

namespace visualization_behaviors
{
PublishMarkerArray::PublishMarkerArray(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : BT::SyncActionNode(name, config), shared_resources_(shared_resources)
{
}

BT::PortsList PublishMarkerArray::providedPorts()
{
  return { BT::InputPort<visualization_msgs::msg::MarkerArray>(kPortIDMarkerArray,
                                                               "MarkerArray to publish verbatim (e.g. a cached array)"),
           BT::InputPort<std::string>(kPortIDTopic, kDefaultMarkerArrayTopic, "Topic to publish the MarkerArray on") };
}

BT::KeyValueVector PublishMarkerArray::metadata()
{
  return { { "description", "Publishes a pre-built visualization MarkerArray on the given topic" },
           { "subcategory", "Visualization" } };
}

BT::NodeStatus PublishMarkerArray::tick()
{
  const auto marker_array = getInput<visualization_msgs::msg::MarkerArray>(kPortIDMarkerArray);
  if (!marker_array.has_value())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Missing required input port [" + std::string(kPortIDMarkerArray) + "]: " + marker_array.error());
    return BT::NodeStatus::FAILURE;
  }

  const std::string topic = getInput<std::string>(kPortIDTopic).value_or(kDefaultMarkerArrayTopic);

  // Lazily create the publisher, keyed to the requested topic so a topic change recreates it.
  if (!marker_publisher_ || marker_publisher_->get_topic_name() != topic)
  {
    marker_publisher_ = shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
  }

  marker_publisher_->publish(marker_array.value());

  spdlog::info("Published MarkerArray of {} markers to '{}'", marker_array.value().markers.size(), topic);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace visualization_behaviors
