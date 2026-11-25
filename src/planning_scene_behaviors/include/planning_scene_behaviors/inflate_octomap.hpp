#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace planning_scene_behaviors
{

/**
 * @brief Inflates the occupied voxels in the planning scene's octomap by a given radius.
 *
 * For each occupied voxel, all voxels within the inflation radius are also marked occupied.
 * The result is output as a new planning scene with the inflated octomap.
 *
 * Data Port Name        | Port Type | Object Type
 * -------------------- | --------- | -----------
 * planning_scene       | input     | moveit_msgs::msg::PlanningScene
 * inflation_radius     | input     | double
 * inflated_scene       | output    | moveit_msgs::msg::PlanningScene
 */
class InflateOctomap : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  InflateOctomap(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace planning_scene_behaviors
