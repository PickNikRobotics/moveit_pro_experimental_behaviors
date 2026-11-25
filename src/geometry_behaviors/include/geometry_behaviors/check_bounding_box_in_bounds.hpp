#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace geometry_behaviors
{

/**
 * @brief Checks if a bounding box is within specified world-frame min/max bounds.
 *
 * This behavior takes a bounding box center pose and dimensions, and verifies that
 * all corners of the bounding box fall within the specified min/max boundary points
 * in the world frame. This accounts for both the size and position of the vehicle.
 *
 * Data Port Name | Port Type | Object Type
 * -------------- | --------- | -----------
 * box_center_pose | input | geometry_msgs::msg::PoseStamped
 * box_dimensions | input | std::vector<double> (x, y, z dimensions)
 * min_point | input | std::vector<double> (x, y, z minimum boundary)
 * max_point | input | std::vector<double> (x, y, z maximum boundary)
 * max_volume_center_pose | output | geometry_msgs::msg::PoseStamped (center of allowed volume)
 * max_volume_dimensions | output | std::vector<double> (dimensions of allowed volume)
 */
class CheckBoundingBoxInBounds : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  CheckBoundingBoxInBounds(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace geometry_behaviors
