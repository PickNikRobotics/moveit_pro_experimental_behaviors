#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace geometry_behaviors
{

/**
 * @brief Computes a PoseStamped at the center of one face of an oriented bounding box,
 * optionally offset outward along the face normal.
 *
 * The face is selected by a signed axis label in the box's local frame:
 *   "+x", "-x" -> faces along box length (dimensions[0])
 *   "+y", "-y" -> faces along box width  (dimensions[1])
 *   "+z", "-z" -> faces along box height (dimensions[2])
 *
 * The face-center position is computed in the box's local frame, then rotated into the input
 * pose's frame using the box orientation. A positive `offset` moves further outward along the
 * face's outward normal; a negative `offset` moves inward toward the box center.
 *
 * The output pose's orientation is copied from `box_center_pose` and its header is preserved.
 *
 * Data Port Name     | Port Type | Object Type
 * ------------------ | --------- | -----------
 * box_center_pose    | input     | geometry_msgs::msg::PoseStamped
 * box_dimensions     | input     | std::vector<double> ([length, width, height])
 * face               | input     | std::string ("+x"|"-x"|"+y"|"-y"|"+z"|"-z")
 * offset             | input     | double (meters; positive = outward)
 * face_pose          | output    | geometry_msgs::msg::PoseStamped
 * face_position_x    | output    | double (face position in the input pose's frame)
 * face_position_y    | output    | double
 * face_position_z    | output    | double
 */
class GetBoundingBoxFacePose : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetBoundingBoxFacePose(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace geometry_behaviors
