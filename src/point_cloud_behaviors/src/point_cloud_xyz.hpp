// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Core>

#include <cstring>
#include <optional>

// Private to this folder's src/: reads x, y and z straight from a PointCloud2, with no PCL, and
// copies whole points as raw bytes so that every other field (color, intensity) is kept.

namespace point_cloud_behaviors::point_cloud_xyz
{
/** @brief Byte offsets of the x, y and z fields inside one point. */
struct XyzLayout
{
  int x_offset = 0;
  int y_offset = 0;
  int z_offset = 0;
};

/**
 * @brief Finds the x, y and z fields of @p cloud.
 * @return nullopt when one of them is missing or is not FLOAT32, or when the cloud is big-endian.
 */
[[nodiscard]] inline std::optional<XyzLayout> findXyzLayout(const sensor_msgs::msg::PointCloud2& cloud)
{
  if (cloud.is_bigendian)
  {
    return std::nullopt;
  }
  std::optional<int> x_offset;
  std::optional<int> y_offset;
  std::optional<int> z_offset;
  for (const auto& field : cloud.fields)
  {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.offset + sizeof(float) > cloud.point_step)
    {
      continue;
    }
    if (field.name == "x")
    {
      x_offset = static_cast<int>(field.offset);
    }
    else if (field.name == "y")
    {
      y_offset = static_cast<int>(field.offset);
    }
    else if (field.name == "z")
    {
      z_offset = static_cast<int>(field.offset);
    }
  }
  if (!x_offset.has_value() || !y_offset.has_value() || !z_offset.has_value())
  {
    return std::nullopt;
  }
  return XyzLayout{ x_offset.value(), y_offset.value(), z_offset.value() };
}

/** @brief True when the data buffer holds every point that width, height, point_step and row_step promise. */
[[nodiscard]] inline bool hasConsistentSize(const sensor_msgs::msg::PointCloud2& cloud)
{
  if (cloud.height == 0 || cloud.width == 0)
  {
    return true;
  }
  const std::size_t row_bytes = static_cast<std::size_t>(cloud.width) * cloud.point_step;
  return cloud.point_step > 0 && cloud.row_step >= row_bytes &&
         cloud.data.size() >= static_cast<std::size_t>(cloud.height - 1) * cloud.row_step + row_bytes;
}

/** @brief Pointer to the first byte of the point at @p row and @p column. */
[[nodiscard]] inline const std::uint8_t* pointData(const sensor_msgs::msg::PointCloud2& cloud, const std::uint32_t row,
                                                   const std::uint32_t column)
{
  return cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
         static_cast<std::size_t>(column) * cloud.point_step;
}

/** @brief The x, y and z of the point at @p data. Values can be NaN. */
[[nodiscard]] inline Eigen::Vector3d readPoint(const std::uint8_t* data, const XyzLayout& layout)
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::memcpy(&x, data + layout.x_offset, sizeof(float));
  std::memcpy(&y, data + layout.y_offset, sizeof(float));
  std::memcpy(&z, data + layout.z_offset, sizeof(float));
  return { static_cast<double>(x), static_cast<double>(y), static_cast<double>(z) };
}

/**
 * @brief An empty unordered cloud with the fields, point size and header of @p like.
 * @details Height is one: a filtered cloud has no image grid left, and a consumer that indexes by
 * pixel must not think it has.
 */
[[nodiscard]] inline sensor_msgs::msg::PointCloud2 makeUnorderedLike(const sensor_msgs::msg::PointCloud2& like)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = like.header;
  cloud.fields = like.fields;
  cloud.is_bigendian = like.is_bigendian;
  cloud.point_step = like.point_step;
  cloud.height = 1;
  cloud.width = 0;
  cloud.row_step = 0;
  cloud.is_dense = like.is_dense;
  return cloud;
}

/** @brief Appends the raw bytes of one point to @p target. */
inline void appendPoint(sensor_msgs::msg::PointCloud2& target, const std::uint8_t* data)
{
  target.data.insert(target.data.end(), data, data + target.point_step);
  target.width += 1;
  target.row_step = target.point_step * target.width;
}
}  // namespace point_cloud_behaviors::point_cloud_xyz
