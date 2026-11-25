#include "point_cloud_behaviors/count_points_in_cloud.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <moveit_pro_behavior_interface/check_for_error.hpp>
#include <moveit_studio_vision/common/get_all_indices.hpp>
#include <moveit_studio_vision/common/select_point_indices.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>

namespace point_cloud_behaviors
{
namespace
{

// Same NaN / near-zero rejection the other point-cloud behaviors in this package apply, so a count
// taken here is comparable with the clouds they emit.
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

// Smallest voxel edge this Behavior counts on, so the voxel indices stay far inside int64_t.
// See docs/AGENT_COMMENTS.md.
constexpr double kMinVoxelSize = 0.001;

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey& k) const
  {
    // Cheap 64-bit mix; the keys are small integers in a bounded volume, so collisions only cost a
    // bucket comparison and never a wrong answer.
    std::size_t h = static_cast<std::size_t>(k.x) * 73856093U;
    h ^= static_cast<std::size_t>(k.y) * 19349663U;
    h ^= static_cast<std::size_t>(k.z) * 83492791U;
    return h;
  }
};

}  // namespace

CountPointsInCloud::CountPointsInCloud(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CountPointsInCloud::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>("point_cloud", "{point_cloud}",
                                                        "Point cloud to measure. Counts whatever it is given, so any "
                                                        "cropping or filtering is the caller's job."),
           BT::InputPort<double>("voxel_size", 0.05,
                                 "Edge length in meters of the voxel grid used for 'voxel_count'. Set to exactly 0 "
                                 "to skip voxel counting, which then reports -1. Any other value must be finite and "
                                 "at least 1 mm."),
           BT::OutputPort<int>("point_count", "{point_count}", "Number of finite, non-near-zero points in the cloud."),
           BT::OutputPort<int>("voxel_count", "{voxel_count}",
                               "Number of distinct occupied voxels at 'voxel_size', or -1 when voxel counting is "
                               "disabled. Prefer this over 'point_count' when comparing captures: it is far less "
                               "sensitive to how close a surface happened to be to a camera."),
           BT::OutputPort<std::string>("point_count_str", "{point_count_str}",
                                       "'point_count' as a string, for ports that take one - a YAML writer, say. A "
                                       "single entry cannot serve both: BT.CPP fixes its type when the tree is built."),
           BT::OutputPort<std::string>("voxel_count_str", "{voxel_count_str}",
                                       "'voxel_count' as a string, for the same reason.") };
}

BT::KeyValueVector CountPointsInCloud::metadata()
{
  return { { "description",
             "Counts the finite points in a point cloud, and the distinct voxels they occupy. Measurement only - "
             "applies no threshold. An empty cloud is SUCCESS with zero counts." },
           { "subcategory", "Perception - 3D Point Cloud" } };
}

BT::NodeStatus CountPointsInCloud::tick()
{
  const auto fail = [this](const std::string& details) {
    getBehaviorContext()->logger->publishFailureMessage(name(), details);
    return BT::NodeStatus::FAILURE;
  };

  const auto cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("point_cloud");
  const auto voxel_size_in = getInput<double>("voxel_size");

  if (const auto error = moveit_pro::behaviors::maybe_error(cloud_msg, voxel_size_in); error)
  {
    return fail("Failed to get required value from input data port: " + error.value());
  }

  const double voxel_size = voxel_size_in.value();
  // std::isfinite first: NaN passes a bare `< 0.0` and would look like "counting disabled", and infinity
  // puts every point in one voxel. Both are reachable from text config.
  if (!std::isfinite(voxel_size) || voxel_size < 0.0 || (voxel_size > 0.0 && voxel_size < kMinVoxelSize))
  {
    return fail("voxel_size must be exactly 0.0 (which disables voxel counting) or a finite value of at least " +
                std::to_string(kMinVoxelSize) + " m, got " + std::to_string(voxel_size));
  }

  // No emptiness check. Zero points is a legitimate measurement of an empty volume, and a caller
  // that needs to tell that apart from a failed capture does so by measuring the uncropped cloud
  // too - see the note in the header.
  const auto cloud = getFilteredPointCloudFromMessage(cloud_msg.value());
  const auto point_count = static_cast<int>(cloud->size());

  int voxel_count = -1;
  if (voxel_size > 0.0)
  {
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
    occupied.reserve(cloud->size());
    for (const auto& p : cloud->points)
    {
      occupied.insert(VoxelKey{ static_cast<std::int64_t>(std::floor(p.x / voxel_size)),
                                static_cast<std::int64_t>(std::floor(p.y / voxel_size)),
                                static_cast<std::int64_t>(std::floor(p.z / voxel_size)) });
    }
    voxel_count = static_cast<int>(occupied.size());
  }

  setOutput("point_count", point_count);
  setOutput("voxel_count", voxel_count);
  setOutput("point_count_str", std::to_string(point_count));
  setOutput("voxel_count_str", std::to_string(voxel_count));

  // The Behavior logger, not RCLCPP_INFO, whose info lines from Behaviors do not reach the log. Keyed on
  // the node name, so a tree that counts more than one cloud stays readable.
  getBehaviorContext()->logger->publishInfoMessage(name(), std::to_string(point_count) + " points, " +
                                                               std::to_string(voxel_count) + " voxels at " +
                                                               std::to_string(voxel_size) + " m");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace point_cloud_behaviors
