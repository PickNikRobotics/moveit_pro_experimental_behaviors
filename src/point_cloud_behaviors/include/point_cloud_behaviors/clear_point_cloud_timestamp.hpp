#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace point_cloud_behaviors
{

/**
 * @brief Zeroes a point cloud's header.stamp so downstream TF lookups resolve against the
 * latest available transforms instead of the cloud's capture time.
 *
 * Use this immediately before handing a cloud to UpdatePlanningSceneService.
 *
 * The octomap updater filters the robot out of a cloud with the link transforms at the cloud stamp. For an
 * old cloud those are gone from the TF buffer, so moving links are not carved out. A zero stamp means "latest
 * available". See docs/AGENT_COMMENTS.md.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * point_cloud | input | sensor_msgs::msg::PointCloud2
 * stamped_point_cloud | output | sensor_msgs::msg::PointCloud2
 */
class ClearPointCloudTimestamp : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ClearPointCloudTimestamp(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace point_cloud_behaviors
