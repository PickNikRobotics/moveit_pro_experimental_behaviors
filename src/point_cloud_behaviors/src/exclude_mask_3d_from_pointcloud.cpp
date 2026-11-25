#include <pcl_conversions/pcl_conversions.h>

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_studio_vision_msgs/msg/mask3_d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tl_expected/expected.hpp>
#include "point_cloud_behaviors/exclude_mask_3d_from_pointcloud.hpp"

namespace
{
inline constexpr auto kDescriptionExcludeMask3DFromPointCloud = R"(
                <p>
                    Returns a point cloud without the points selected by a 3D mask.
                </p>
            )";
constexpr auto kPortIDPointCloud = "point_cloud";
constexpr auto kPortIDMask3D = "mask3d";
constexpr auto kPortIDPointCloudExcluded = "point_cloud_excluded";

}  // namespace

namespace point_cloud_behaviors
{

ExcludeMask3DFromPointCloud::ExcludeMask3DFromPointCloud(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList ExcludeMask3DFromPointCloud::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}", "Input point cloud."),
           BT::InputPort<moveit_studio_vision_msgs::msg::Mask3D>(kPortIDMask3D, "{mask3d}",
                                                                 "3D mask selecting which points to exclude from the "
                                                                 "point cloud to the output."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloudExcluded, "{point_cloud_excluded}",
                                                         "Point cloud points with the masked points removed.") };
}

BT::KeyValueVector ExcludeMask3DFromPointCloud::metadata()
{
  return { { "subcategory", "Perception - 3D Point Cloud" },
           { "description", kDescriptionExcludeMask3DFromPointCloud } };
}

tl::expected<bool, std::string> ExcludeMask3DFromPointCloud::doWork()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud),
                                               getInput<moveit_studio_vision_msgs::msg::Mask3D>(kPortIDMask3D));

  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }

  const auto& [point_cloud_msg, mask3d_msg] = ports.value();

  if (mask3d_msg.point_indices.empty())
  {
    return tl::make_unexpected("The 3D mask is empty.");
  }

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  pcl::fromROSMsg(point_cloud_msg, point_cloud);

  // Copy the point cloud fragment that EXCLUDES the 3D mask (i.e., the complement).
  // Build a fast lookup table of masked indices (bounds-checked).
  const std::size_t N = point_cloud.size();
  std::vector<uint8_t> is_masked(N, 0);
  is_masked.shrink_to_fit();  // keep memory tight

  for (const int idx_raw : mask3d_msg.point_indices)
  {
    if (idx_raw >= 0 && static_cast<std::size_t>(idx_raw) < N)
    {
      is_masked[static_cast<std::size_t>(idx_raw)] = 1;
    }
  }

  // Collect indices to KEEP: all points not in the mask.
  pcl::PointIndices pcl_indices_keep;
  pcl_indices_keep.header = point_cloud.header;
  pcl_indices_keep.indices.reserve(N);

  for (std::size_t i = 0; i < N; ++i)
  {
    if (!is_masked[i])
    {
      pcl_indices_keep.indices.push_back(static_cast<int>(i));
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud_fragment;
  pcl::copyPointCloud(point_cloud, pcl_indices_keep, point_cloud_fragment);

  sensor_msgs::msg::PointCloud2 point_cloud_fragment_msg;
  pcl::toROSMsg(point_cloud_fragment, point_cloud_fragment_msg);

  setOutput(kPortIDPointCloudExcluded, point_cloud_fragment_msg);

  return true;
}
}  // namespace point_cloud_behaviors
