// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "point_cloud_behaviors/poisson_reconstruct.hpp"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"

namespace point_cloud_behaviors
{
namespace
{
constexpr auto kPortInputCloud = "input_cloud";
constexpr auto kPortPoissonDepth = "poisson_depth";
constexpr auto kPortNormalRadius = "normal_radius";
constexpr auto kPortTrimDistance = "trim_distance";
constexpr auto kPortFillThreshold = "fill_threshold";
constexpr auto kPortScannedColor = "scanned_color";
constexpr auto kPortFilledColor = "filled_color";
constexpr auto kPortOutputCloud = "output_cloud";

// Parse "r;g;b" (0-255) into a uint8 triple. Returns false on malformed input.
bool parseColor(const std::string& s, std::array<std::uint8_t, 3>& out)
{
  std::stringstream ss(s);
  std::string tok;
  std::vector<int> vals;
  while (std::getline(ss, tok, ';'))
  {
    try
    {
      vals.push_back(std::stoi(tok));
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
  for (int i = 0; i < 3; ++i)
  {
    out[i] = static_cast<std::uint8_t>(std::clamp(vals[i], 0, 255));
  }
  return true;
}

pcl::PointXYZRGB makeColored(float x, float y, float z, const std::array<std::uint8_t, 3>& c)
{
  pcl::PointXYZRGB p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.r = c[0];
  p.g = c[1];
  p.b = c[2];
  return p;
}
}  // namespace

PoissonReconstruct::PoissonReconstruct(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList PoissonReconstruct::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortInputCloud, "{input_cloud}", "Point cloud to reconstruct."),
           BT::InputPort<int>(kPortPoissonDepth, 9, "Poisson octree depth (higher = finer, slower)."),
           BT::InputPort<double>(kPortNormalRadius, 0.03, "Normal-estimation neighborhood radius (m)."),
           BT::InputPort<double>(kPortTrimDistance, 0.025,
                                 "Drop surface points farther than this from any measured point (m)."),
           BT::InputPort<double>(kPortFillThreshold, 0.008,
                                 "Surface points farther than this from a measured point are colored as filled (m)."),
           BT::InputPort<std::string>(kPortScannedColor, "200;200;200", "RGB of measured/scanned points 'r;g;b'."),
           BT::InputPort<std::string>(kPortFilledColor, "255;60;60", "RGB of filled/interpolated points 'r;g;b'."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>(kPortOutputCloud, "{output_cloud}",
                                                         "Measured points (scanned_color) + filled points "
                                                         "(filled_color), as PointXYZRGB.") };
}

BT::KeyValueVector PoissonReconstruct::metadata()
{
  return { { "description", "Poisson surface reconstruction; measured vs filled points colored differently." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus PoissonReconstruct::tick()
{
  const auto cloud_result = getInput<sensor_msgs::msg::PointCloud2>(kPortInputCloud);
  if (!cloud_result)
  {
    spdlog::error("PoissonReconstruct: failed to get '{}': {}", kPortInputCloud, cloud_result.error());
    return BT::NodeStatus::FAILURE;
  }
  const int depth = getInput<int>(kPortPoissonDepth).value_or(9);
  const double normal_radius = getInput<double>(kPortNormalRadius).value_or(0.03);
  const double trim_distance = getInput<double>(kPortTrimDistance).value_or(0.025);
  const double fill_threshold = getInput<double>(kPortFillThreshold).value_or(0.008);

  std::array<std::uint8_t, 3> scanned_color{ 200, 200, 200 };
  std::array<std::uint8_t, 3> filled_color{ 255, 60, 60 };
  if (!parseColor(getInput<std::string>(kPortScannedColor).value_or("200;200;200"), scanned_color) ||
      !parseColor(getInput<std::string>(kPortFilledColor).value_or("255;60;60"), filled_color))
  {
    spdlog::error("PoissonReconstruct: could not parse a color port (expected 'r;g;b').");
    return BT::NodeStatus::FAILURE;
  }

  const sensor_msgs::msg::PointCloud2& in_msg = cloud_result.value();
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(in_msg, *in_cloud);
  if (in_cloud->size() < 4)
  {
    spdlog::error("PoissonReconstruct: input cloud too small ({} points).", in_cloud->size());
    return BT::NodeStatus::FAILURE;
  }

  // Oriented normals: scanned from above, so orient toward a viewpoint high in +Z.
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(in_cloud);
  ne.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
  ne.setRadiusSearch(normal_radius);
  ne.setViewPoint(0.0F, 0.0F, 10.0F);
  pcl::PointCloud<pcl::Normal>::Ptr normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  pcl::concatenateFields(*in_cloud, *normals, *cloud_normals);

  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(depth);
  poisson.setInputCloud(cloud_normals);
  pcl::PolygonMesh mesh;
  try
  {
    poisson.reconstruct(mesh);
  }
  catch (const pcl::PCLException& e)
  {
    spdlog::error("PoissonReconstruct: Poisson failed: {}", e.what());
    return BT::NodeStatus::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr verts = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *verts);
  if (verts->empty())
  {
    spdlog::error("PoissonReconstruct: Poisson produced no vertices.");
    return BT::NodeStatus::FAILURE;
  }

  // KD-tree on the measured cloud to classify each surface vertex.
  pcl::search::KdTree<pcl::PointXYZ> kdt;
  kdt.setInputCloud(in_cloud);
  const double trim_sq = trim_distance * trim_distance;
  const double fill_sq = fill_threshold * fill_threshold;

  pcl::PointCloud<pcl::PointXYZRGB> out;
  out.reserve(in_cloud->size() + verts->size());
  // Measured points: keep the real scan, colored as scanned.
  for (const auto& p : in_cloud->points)
  {
    out.push_back(makeColored(p.x, p.y, p.z, scanned_color));
  }
  // Poisson vertices in the fill band (beyond a measured point but still supported): filled color.
  std::vector<int> idx(1);
  std::vector<float> d2(1);
  std::size_t filled = 0;
  for (const auto& v : verts->points)
  {
    if (kdt.nearestKSearch(v, 1, idx, d2) < 1)
    {
      continue;
    }
    if (d2[0] > fill_sq && d2[0] <= trim_sq)
    {
      out.push_back(makeColored(v.x, v.y, v.z, filled_color));
      ++filled;
    }
  }

  spdlog::info("PoissonReconstruct: {} measured + {} filled = {} points (depth={}, trim={:.3f}m).", in_cloud->size(),
               filled, out.size(), depth, trim_distance);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(out, out_msg);
  out_msg.header = in_msg.header;
  setOutput(kPortOutputCloud, out_msg);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace point_cloud_behaviors
