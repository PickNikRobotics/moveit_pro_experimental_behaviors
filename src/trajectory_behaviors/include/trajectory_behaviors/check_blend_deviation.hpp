#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace trajectory_behaviors
{

/**
 * @brief Rejects a blended trajectory that strays too far (in joint space) from the original
 * un-blended path, evaluated only within the blended regions.
 *
 * BlendJointTrajectories bounds neither collision nor path deviation, so a large blending radius can loop
 * away from the inputs. This checks each blend zone point against the reference point at the same
 * normalized time. `blend_radius` must match BlendJointTrajectories. See docs/AGENT_COMMENTS.md.
 *
 * All trajectories must contain the same set of joints, but their joint_names orderings may
 * differ; each original trajectory is remapped into blended_trajectory's joint order before
 * comparison.
 *
 * | Data Port Name        | Port Type | Object Type                                        |
 * | --------------------- | --------- | -------------------------------------------------- |
 * | original_trajectories | Input     | std::vector<trajectory_msgs::msg::JointTrajectory> |
 * | blended_trajectory    | Input     | trajectory_msgs::msg::JointTrajectory              |
 * | max_joint_deviation   | Input     | double                                             |
 * | blend_radius          | Input     | double                                             |
 * | measured_deviation    | Output    | double                                             |
 */
class CheckBlendDeviation : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  CheckBlendDeviation(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace trajectory_behaviors
