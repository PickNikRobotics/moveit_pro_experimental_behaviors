// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "image_behaviors/mark_points_on_image.hpp"

// Jazzy ships cv_bridge.hpp; Humble ships only cv_bridge.h.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
constexpr auto kPortIDImage = "image";
constexpr auto kPortIDPoints = "points";
constexpr auto kPortIDTopic = "topic";
constexpr auto kPortIDOutputImage = "output_image";

inline constexpr auto kDescription = R"(
    <p>Draws visible markers on an image at each point location and publishes the result.</p>
    <p>Input points are in normalized image UV coordinates (x,y in [0,1]).
    A crosshair marker is drawn at each corresponding pixel location and the
    annotated image is published to the specified topic.</p>
)";

constexpr int kMarkerRadius = 10;
constexpr int kCrosshairExtension = 5;
constexpr int kOutlineThickness = 2;
}  // namespace

namespace image_behaviors
{

MarkPointsOnImage::MarkPointsOnImage(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList MarkPointsOnImage::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage, "{image}", "Input image to annotate."),
    BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortIDPoints, "{points}",
                                                                 "Points with x,y in [0,1] as image UV coordinates."),
    BT::InputPort<std::string>(kPortIDTopic, "/marked_image", "Topic to publish the annotated image to."),
    BT::OutputPort<sensor_msgs::msg::Image>(kPortIDOutputImage, "{marked_image}", "The annotated image."),
  };
}

BT::KeyValueVector MarkPointsOnImage::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Visualization" } };
}

BT::NodeStatus MarkPointsOnImage::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>(kPortIDImage),
                                               getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortIDPoints));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required inputs: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [image, points] = ports.value();
  const std::string topic = getInput<std::string>(kPortIDTopic).value_or("/marked_image");

  // Create or update the publisher if the topic changed.
  if (!publisher_ || current_topic_ != topic)
  {
    publisher_ =
        getBehaviorContext()->node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1).transient_local());
    current_topic_ = topic;
  }

  // Convert to a BGR8 OpenCV image for drawing.
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception& e)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), std::string("cv_bridge conversion failed: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }

  const int image_width = cv_ptr->image.cols;
  const int image_height = cv_ptr->image.rows;

  if (image_width == 0 || image_height == 0)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Image has zero dimensions.");
    return BT::NodeStatus::FAILURE;
  }

  const int arm = kMarkerRadius + kCrosshairExtension;

  for (const auto& point : points)
  {
    // Convert normalized UV [0,1] to pixel coordinates.
    const int px = std::clamp(static_cast<int>(std::round(point.point.x * (image_width - 1))), 0, image_width - 1);
    const int py = std::clamp(static_cast<int>(std::round(point.point.y * (image_height - 1))), 0, image_height - 1);
    const cv::Point center(px, py);

    // Draw a white-outlined red circle with a white crosshair.
    cv::circle(cv_ptr->image, center, kMarkerRadius + kOutlineThickness, cv::Scalar(255, 255, 255), kOutlineThickness);
    cv::circle(cv_ptr->image, center, kMarkerRadius, cv::Scalar(0, 0, 255), -1);
    cv::line(cv_ptr->image, { px - arm, py }, { px + arm, py }, cv::Scalar(255, 255, 255), kOutlineThickness);
    cv::line(cv_ptr->image, { px, py - arm }, { px, py + arm }, cv::Scalar(255, 255, 255), kOutlineThickness);
  }

  // Publish and output.
  const auto output_msg = cv_ptr->toImageMsg();
  publisher_->publish(*output_msg);
  setOutput(kPortIDOutputImage, *output_msg);

  getBehaviorContext()->logger->publishInfoMessage(name(), "Marked " + std::to_string(points.size()) +
                                                               " point(s) on topic '" + topic + "'.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace image_behaviors
