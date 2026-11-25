#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace point_cloud_behaviors
{

/**
 * @brief Reports how much geometry a point cloud carries, as a raw point count and as a count of
 *        distinct occupied voxels.
 *
 * Measurement only: it applies no threshold. The caller compares the counts against its own limit.
 *
 * Only finite points that are not near the origin are counted, as in `PclNnFilter`. An empty cloud is
 * SUCCESS with zero counts, not FAILURE. See docs/AGENT_COMMENTS.md.
 *
 * Each count is output as an int (to compare in a Precondition) and as a string (to write to YAML),
 * because one blackboard entry has one type for the whole tree.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * point_cloud | input | sensor_msgs::msg::PointCloud2
 * voxel_size | input | double   (exactly 0 disables counting; otherwise finite and >= 1 mm)
 * point_count | output | int
 * voxel_count | output | int
 * point_count_str | output | std::string
 * voxel_count_str | output | std::string
 */
class CountPointsInCloud : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  CountPointsInCloud(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace point_cloud_behaviors
