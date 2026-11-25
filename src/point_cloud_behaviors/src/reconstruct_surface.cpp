// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "point_cloud_behaviors/reconstruct_surface.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

#include "spdlog/spdlog.h"

namespace point_cloud_behaviors
{
namespace
{
constexpr auto kPortInputCloud = "input_cloud";
constexpr auto kPortSearchRadius = "search_radius";
constexpr auto kPortPolynomialOrder = "polynomial_order";
constexpr auto kPortUpsamplingMethod = "upsampling_method";
constexpr auto kPortVoxelSize = "voxel_size";
constexpr auto kPortDilationIterations = "dilation_iterations";
constexpr auto kPortOutputCloud = "output_cloud";

using Mls = pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>;
}  // namespace

ReconstructSurface::ReconstructSurface(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ReconstructSurface::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortInputCloud, "{input_cloud}", "Point cloud to reconstruct."),
           BT::InputPort<double>(kPortSearchRadius, 0.03, "MLS neighborhood radius (m)."),
           BT::InputPort<int>(kPortPolynomialOrder, 2, "Local polynomial order."),
           BT::InputPort<std::string>(kPortUpsamplingMethod, "voxel_grid_dilation",
                                      "voxel_grid_dilation | sample_local_plane | none."),
           BT::InputPort<double>(kPortVoxelSize, 0.01, "VOXEL_GRID_DILATION voxel size (m)."),
           BT::InputPort<int>(kPortDilationIterations, 1, "VOXEL_GRID_DILATION dilation passes."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>(kPortOutputCloud, "{output_cloud}",
                                                         "Reconstructed point cloud (PointXYZ).") };
}

BT::KeyValueVector ReconstructSurface::metadata()
{
  return { { "description", "Surface-following hole fill / smoothing via Moving Least Squares." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus ReconstructSurface::tick()
{
  const auto cloud_result = getInput<sensor_msgs::msg::PointCloud2>(kPortInputCloud);
  if (!cloud_result)
  {
    spdlog::error("ReconstructSurface: failed to get '{}': {}", kPortInputCloud, cloud_result.error());
    return BT::NodeStatus::FAILURE;
  }
  const double search_radius = getInput<double>(kPortSearchRadius).value_or(0.03);
  const int polynomial_order = getInput<int>(kPortPolynomialOrder).value_or(2);
  const std::string method = getInput<std::string>(kPortUpsamplingMethod).value_or("voxel_grid_dilation");
  const double voxel_size = getInput<double>(kPortVoxelSize).value_or(0.01);
  const int dilation_iterations = getInput<int>(kPortDilationIterations).value_or(1);

  if (search_radius <= 0.0)
  {
    spdlog::error("ReconstructSurface: search_radius must be > 0 (got {}).", search_radius);
    return BT::NodeStatus::FAILURE;
  }

  const sensor_msgs::msg::PointCloud2& in_msg = cloud_result.value();
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(in_msg, *in_cloud);
  if (in_cloud->empty())
  {
    spdlog::error("ReconstructSurface: input cloud is empty.");
    return BT::NodeStatus::FAILURE;
  }

  Mls mls;
  mls.setInputCloud(in_cloud);
  mls.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
  mls.setSearchRadius(search_radius);
  mls.setPolynomialOrder(polynomial_order);
  mls.setComputeNormals(false);

  if (method == "voxel_grid_dilation")
  {
    mls.setUpsamplingMethod(Mls::VOXEL_GRID_DILATION);
    mls.setDilationVoxelSize(static_cast<float>(voxel_size));
    mls.setDilationIterations(dilation_iterations);
  }
  else if (method == "sample_local_plane")
  {
    mls.setUpsamplingMethod(Mls::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(search_radius);
    mls.setUpsamplingStepSize(voxel_size);
  }
  else if (method == "none")
  {
    mls.setUpsamplingMethod(Mls::NONE);
  }
  else
  {
    spdlog::error("ReconstructSurface: unknown upsampling_method '{}'.", method);
    return BT::NodeStatus::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ> out_cloud;
  try
  {
    mls.process(out_cloud);
  }
  catch (const pcl::PCLException& e)
  {
    spdlog::error("ReconstructSurface: MLS failed: {}", e.what());
    return BT::NodeStatus::FAILURE;
  }

  if (out_cloud.empty())
  {
    spdlog::error("ReconstructSurface: MLS produced an empty cloud (try a larger search_radius or coarser "
                  "voxel_size).");
    return BT::NodeStatus::FAILURE;
  }

  spdlog::info("ReconstructSurface: {} -> {} points (method={}, radius={:.3f}m).", in_cloud->size(), out_cloud.size(),
               method, search_radius);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(out_cloud, out_msg);
  out_msg.header = in_msg.header;
  setOutput(kPortOutputCloud, out_msg);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace point_cloud_behaviors
