// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "image_behaviors/find_mask_index_for_point.hpp"

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
constexpr auto kPortIDPoint = "point";
constexpr auto kPortIDImage = "image";
constexpr auto kPortIDMasks2D = "masks2d";
constexpr auto kPortIDMaskIndex = "mask_index";

inline constexpr auto kDescription = R"(
    <p>Finds the index of the first 2D mask that contains a given point (normalized image UV coordinates).</p>
    <p>The input PointStamped has x,y in [0,1] representing UV coordinates in the image. The behavior
    walks the masks in input order and returns the index of the first mask whose pixel value at the
    point is non-zero. Fails if the point is outside every mask.</p>
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

  spdlog::warn("FindMaskIndexForPoint: unsupported mask encoding '{}', treating pixel as outside mask.", encoding);
  return false;
}
}  // namespace

namespace image_behaviors
{

FindMaskIndexForPoint::FindMaskIndexForPoint(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList FindMaskIndexForPoint::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PointStamped>(kPortIDPoint, "{point}",
                                                    "Point with x,y in [0,1] as image UV coordinates."),
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage, "{image}", "Image used to determine pixel dimensions."),
    BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortIDMasks2D, "{masks2d}", "2D masks."),
    BT::OutputPort<int>(kPortIDMaskIndex, "{mask_index}", "Index of the first mask in masks2d that contains the point."),
  };
}

BT::KeyValueVector FindMaskIndexForPoint::metadata()
{
  return { { "description", kDescription }, { "subcategory", "Perception - 2D" } };
}

BT::NodeStatus FindMaskIndexForPoint::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PointStamped>(kPortIDPoint), getInput<sensor_msgs::msg::Image>(kPortIDImage),
      getInput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortIDMasks2D));

  if (!ports.has_value())
  {
    spdlog::error("FindMaskIndexForPoint: failed to get required inputs: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [point, image, masks] = ports.value();

  if (masks.empty())
  {
    spdlog::error("FindMaskIndexForPoint: input masks vector is empty.");
    return BT::NodeStatus::FAILURE;
  }

  const int image_width = static_cast<int>(image.width);
  const int image_height = static_cast<int>(image.height);

  if (image_width == 0 || image_height == 0)
  {
    spdlog::error("FindMaskIndexForPoint: image has zero dimensions ({}x{}).", image_width, image_height);
    return BT::NodeStatus::FAILURE;
  }

  const int px = std::clamp(static_cast<int>(std::round(point.point.x * (image_width - 1))), 0, image_width - 1);
  const int py = std::clamp(static_cast<int>(std::round(point.point.y * (image_height - 1))), 0, image_height - 1);

  for (int i = 0; i < static_cast<int>(masks.size()); ++i)
  {
    if (isPixelInMask(px, py, masks[i]))
    {
      setOutput(kPortIDMaskIndex, i);
      spdlog::info("FindMaskIndexForPoint: point ({:.3f}, {:.3f}) → pixel ({}, {}) is inside mask {}.", point.point.x,
                   point.point.y, px, py, i);
      return BT::NodeStatus::SUCCESS;
    }
  }

  spdlog::error("FindMaskIndexForPoint: point ({:.3f}, {:.3f}) → pixel ({}, {}) is outside all {} masks.",
                point.point.x, point.point.y, px, py, masks.size());
  return BT::NodeStatus::FAILURE;
}

}  // namespace image_behaviors
