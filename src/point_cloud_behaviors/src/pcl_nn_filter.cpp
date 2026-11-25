#include "point_cloud_behaviors/pcl_nn_filter.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <moveit_pro_behavior_interface/check_for_error.hpp>
#include <moveit_studio_vision/common/get_all_indices.hpp>
#include <moveit_studio_vision/common/select_point_indices.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>

#include <spdlog/spdlog.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tl_expected/expected.hpp>

#include <chrono>

namespace point_cloud_behaviors
{
namespace
{

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
getFilteredPointCloudFromMessage(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  const auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(cloud_msg, *cloud);

  const pcl::PointIndices valid_indices =
      moveit_studio::selectPointIndices(*cloud, moveit_studio::point_cloud_tools::getAllIndices(cloud),
                                        moveit_studio::NeitherNanNorNearZeroPointValidator<pcl::PointXYZRGB>);

  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*cloud, valid_indices, *filtered_cloud);
  return filtered_cloud;
}

}  // namespace

ExampleRadiusNeighborsFilter::ExampleRadiusNeighborsFilter(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList ExampleRadiusNeighborsFilter::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::PointCloud2>("point_cloud", "{point_cloud}", "Input point cloud to filter."),
    BT::InputPort<double>("radius", 0.05, "Neighbor search radius (meters)."),
    BT::InputPort<int>("min_neighbors", 2,
                       "Minimum number of neighbors required within `radius` (excluding the query point)."),
    BT::OutputPort<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", "{filtered_point_cloud}",
                                                  "Filtered point cloud with sparse points removed."),
  };
}

BT::KeyValueVector ExampleRadiusNeighborsFilter::metadata()
{
  return {
    { "subcategory", "Perception - 3D Point Cloud" },
    { "description",
      "Removes points that do not have at least N neighbors within a radius using a KD-tree radius search." },
  };
}

tl::expected<bool, std::string> ExampleRadiusNeighborsFilter::doWork()
{
  const auto t_start = std::chrono::steady_clock::now();

  const auto cloud_msg_in = getInput<sensor_msgs::msg::PointCloud2>("point_cloud");
  const auto radius_in = getInput<double>("radius");
  const auto min_neighbors_in = getInput<int>("min_neighbors");

  if (const auto error = moveit_pro::behaviors::maybe_error(cloud_msg_in, radius_in, min_neighbors_in); error)
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  const double radius = radius_in.value();
  const int min_neighbors = min_neighbors_in.value();
  spdlog::info("PclNnFilter: input {} bytes (width={}, height={}), params radius={:.4f}m, min_neighbors={}",
               cloud_msg_in->data.size(), cloud_msg_in->width, cloud_msg_in->height, radius, min_neighbors);

  if (!(radius > 0.0))
  {
    return tl::make_unexpected("radius must be > 0.0");
  }
  if (min_neighbors < 0)
  {
    return tl::make_unexpected("min_neighbors must be >= 0");
  }

  const auto cloud = getFilteredPointCloudFromMessage(cloud_msg_in.value());
  spdlog::info("PclNnFilter: after NaN/near-zero drop: {} points", cloud->points.size());
  if (cloud->empty())
  {
    return tl::make_unexpected("Input point cloud has no valid points");
  }

  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);

  pcl::PointIndices keep;
  keep.indices.reserve(cloud->size());

  std::vector<int> nn_indices;
  std::vector<float> nn_sq_dists;

  long total_neighbors = 0;
  int min_nn = std::numeric_limits<int>::max();
  int max_nn = 0;

  for (std::size_t i = 0; i < cloud->points.size(); ++i)
  {
    nn_indices.clear();
    nn_sq_dists.clear();

    // radiusSearch includes the query point itself as one of the returned indices (distance 0).
    const int found = tree.radiusSearch(cloud->points[i], radius, nn_indices, nn_sq_dists);

    // neighbors excluding "self"
    const int neighbors_excluding_self = (found > 0) ? (static_cast<int>(nn_indices.size()) - 1) : 0;
    total_neighbors += neighbors_excluding_self;
    min_nn = std::min(min_nn, neighbors_excluding_self);
    max_nn = std::max(max_nn, neighbors_excluding_self);

    if (neighbors_excluding_self >= min_neighbors)
    {
      keep.indices.push_back(static_cast<int>(i));
    }
  }

  const double avg_nn =
      cloud->points.empty() ? 0.0 : static_cast<double>(total_neighbors) / static_cast<double>(cloud->points.size());
  spdlog::info("PclNnFilter: neighbor-count stats (radius={:.4f}): min={}, max={}, avg={:.2f} per point", radius,
               (cloud->points.empty() ? 0 : min_nn), max_nn, avg_nn);

  pcl::PointCloud<pcl::PointXYZRGB> filtered;
  filtered.reserve(keep.indices.size());
  pcl::copyPointCloud(*cloud, keep, filtered);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(filtered, out_msg);
  out_msg.header = cloud_msg_in.value().header;  // preserve frame + stamp

  const auto t_end = std::chrono::steady_clock::now();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  spdlog::info("PclNnFilter: kept {}/{} points ({:.1f}%), output {} bytes, took {:.1f} ms", keep.indices.size(),
               cloud->points.size(), cloud->points.empty() ? 0.0 : 100.0 * keep.indices.size() / cloud->points.size(),
               out_msg.data.size(), elapsed_ms);

  setOutput("filtered_point_cloud", out_msg);
  return true;
}

}  // namespace point_cloud_behaviors
