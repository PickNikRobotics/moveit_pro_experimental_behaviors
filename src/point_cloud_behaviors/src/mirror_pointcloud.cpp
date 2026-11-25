// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "point_cloud_behaviors/mirror_pointcloud.hpp"

#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <vector>

#include "spdlog/spdlog.h"

namespace point_cloud_behaviors
{
namespace
{
constexpr auto kPortInputCloud = "input_cloud";
constexpr auto kPortOutputCloud = "output_cloud";
constexpr auto kPortPlanePoint = "plane_point";
constexpr auto kPortPlaneNormal = "plane_normal";

// Parse an "x;y;z" string into an Eigen vector. Returns false on malformed input.
bool parseVec3(const std::string& s, Eigen::Vector3d& out)
{
  std::stringstream ss(s);
  std::string tok;
  std::vector<double> vals;
  while (std::getline(ss, tok, ';'))
  {
    try
    {
      vals.push_back(std::stod(tok));
    }
    catch (const std::exception&)
    {
      return false;
    }
  }
  if (vals.size() != 3)
  {
    return false;
  }
  out = Eigen::Vector3d(vals[0], vals[1], vals[2]);
  return true;
}
}  // namespace

MirrorPointCloud::MirrorPointCloud(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList MirrorPointCloud::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortInputCloud, "{input_cloud}",
                                                        "Point cloud to reflect across the plane."),
           BT::InputPort<std::string>(kPortPlanePoint, "0;0;0", "Point on the mirror plane as 'x;y;z'."),
           BT::InputPort<std::string>(kPortPlaneNormal, "0;1;0", "Mirror plane normal as 'x;y;z' (default Y=0 plane)."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>(kPortOutputCloud, "{output_cloud}",
                                                         "Reflected point cloud (xyz reflected, other fields kept).") };
}

BT::KeyValueVector MirrorPointCloud::metadata()
{
  return { { "description", "Reflect a point cloud across a plane (default plane Y=0)." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus MirrorPointCloud::tick()
{
  const auto cloud_result = getInput<sensor_msgs::msg::PointCloud2>(kPortInputCloud);
  if (!cloud_result)
  {
    spdlog::error("MirrorPointCloud: failed to get '{}': {}", kPortInputCloud, cloud_result.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto point_str = getInput<std::string>(kPortPlanePoint).value_or("0;0;0");
  const auto normal_str = getInput<std::string>(kPortPlaneNormal).value_or("0;1;0");

  Eigen::Vector3d q;
  Eigen::Vector3d n;
  if (!parseVec3(point_str, q) || !parseVec3(normal_str, n))
  {
    spdlog::error("MirrorPointCloud: could not parse plane_point '{}' / plane_normal '{}' (expected 'x;y;z').",
                  point_str, normal_str);
    return BT::NodeStatus::FAILURE;
  }
  if (n.norm() < 1e-9)
  {
    spdlog::error("MirrorPointCloud: plane_normal has zero length.");
    return BT::NodeStatus::FAILURE;
  }
  n.normalize();

  // Copy preserves the layout and all non-xyz fields (e.g. rgb); we only reflect xyz.
  sensor_msgs::msg::PointCloud2 out = cloud_result.value();
  if (out.data.empty())
  {
    spdlog::error("MirrorPointCloud: input cloud is empty.");
    return BT::NodeStatus::FAILURE;
  }

  sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz)
  {
    const Eigen::Vector3d p(*ix, *iy, *iz);
    const Eigen::Vector3d reflected = p - 2.0 * (p - q).dot(n) * n;
    *ix = static_cast<float>(reflected.x());
    *iy = static_cast<float>(reflected.y());
    *iz = static_cast<float>(reflected.z());
  }

  setOutput(kPortOutputCloud, out);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace point_cloud_behaviors
