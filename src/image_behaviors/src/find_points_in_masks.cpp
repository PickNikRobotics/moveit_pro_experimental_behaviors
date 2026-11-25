// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "image_behaviors/find_points_in_masks.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
constexpr auto kPortIDPoints = "points";
constexpr auto kPortIDImage = "image";
constexpr auto kPortIDMasks2D = "masks2d";
constexpr auto kPortIDPointIndices = "point_indices";

inline constexpr auto kDescription = R"(
    <p>Finds which points (in normalized image UV coordinates) fall inside any of the given 2D masks.</p>
    <p>Each input PointStamped has x,y in [0,1] representing UV coordinates in the image.
    The behavior checks each point against every mask and returns the indices of points
    that fall inside at least one mask. Fails if no points are inside any mask.</p>
)";

/**
 * @brief Check whether a pixel coordinate falls inside a mask and the mask value is non-zero.
 */
bool isPixelInMask(int px, int py, const moveit_studio_vision_msgs::msg::Mask2D& mask)
{
  const int local_x = px - mask.x;
  const int local_y = py - mask.y;

  if (local_x < 0 || local_x >= static_cast<int>(mask.pixels.width) || local_y < 0 ||
      local_y >= static_cast<int>(mask.pixels.height))
  {
    return false;
  }

  const auto& encoding = mask.pixels.encoding;

  if (encoding == "mono8" || encoding == "8UC1")
  {
    const size_t offset = local_y * mask.pixels.step + local_x;
    return mask.pixels.data[offset] != 0;
  }
  else if (encoding == "32FC1")
  {
    const size_t offset = local_y * mask.pixels.step + local_x * sizeof(float);
    float value;
    std::memcpy(&value, &mask.pixels.data[offset], sizeof(float));
    return value > 0.0f;
  }
  else if (encoding == "16UC1")
  {
    const size_t offset = local_y * mask.pixels.step + local_x * sizeof(uint16_t);
    uint16_t value;
    std::memcpy(&value, &mask.pixels.data[offset], sizeof(uint16_t));
    return value != 0;
  }

  spdlog::warn("FindPointsInMasks: unsupported mask encoding '{}', treating pixel as outside mask.", encoding);
  return false;
}
}  // namespace

namespace image_behaviors
{

FindPointsInMasks::FindPointsInMasks(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList FindPointsInMasks::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortIDPoints, "{points}",
                                                                 "Points with x,y in [0,1] as image UV coordinates."),
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage, "{image}", "Image used to determine pixel dimensions."),
    BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortIDMasks2D, "{masks2d}", "2D masks."),
    BT::OutputPort<std::vector<int>>(kPortIDPointIndices, "{point_indices}",
                                     "Indices of points that fall inside at least one mask."),
  };
}

BT::KeyValueVector FindPointsInMasks::metadata()
{
  return { { "description", kDescription }, { "subcategory", "Perception - 2D" } };
}

BT::NodeStatus FindPointsInMasks::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortIDPoints),
      getInput<sensor_msgs::msg::Image>(kPortIDImage),
      getInput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortIDMasks2D));

  if (!ports.has_value())
  {
    spdlog::error("FindPointsInMasks: failed to get required inputs: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [points, image, masks] = ports.value();

  if (points.empty())
  {
    spdlog::error("FindPointsInMasks: input points vector is empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (masks.empty())
  {
    spdlog::error("FindPointsInMasks: input masks vector is empty.");
    return BT::NodeStatus::FAILURE;
  }

  const int image_width = static_cast<int>(image.width);
  const int image_height = static_cast<int>(image.height);

  if (image_width == 0 || image_height == 0)
  {
    spdlog::error("FindPointsInMasks: image has zero dimensions ({}x{}).", image_width, image_height);
    return BT::NodeStatus::FAILURE;
  }

  std::vector<int> point_indices;

  for (int i = 0; i < static_cast<int>(points.size()); ++i)
  {
    const auto& point = points[i];

    // Convert normalized UV [0,1] to pixel coordinates.
    const int px = std::clamp(static_cast<int>(std::round(point.point.x * (image_width - 1))), 0, image_width - 1);
    const int py = std::clamp(static_cast<int>(std::round(point.point.y * (image_height - 1))), 0, image_height - 1);

    for (const auto& mask : masks)
    {
      if (isPixelInMask(px, py, mask))
      {
        point_indices.push_back(i);
        break;  // Point is inside at least one mask, no need to check the rest.
      }
    }
  }

  if (point_indices.empty())
  {
    spdlog::error("FindPointsInMasks: no points fell inside any mask.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDPointIndices, point_indices);

  spdlog::info("FindPointsInMasks: {} of {} points are inside masks.", point_indices.size(), points.size());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace image_behaviors
