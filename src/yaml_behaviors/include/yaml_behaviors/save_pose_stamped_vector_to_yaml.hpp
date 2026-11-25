// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>
#include <vector>

#include <behaviortree_cpp/behavior_tree.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace yaml_behaviors
{
/**
 * @brief Save a std::vector<geometry_msgs::msg::PoseStamped> (e.g. a coverage path) to a multi-document
 * YAML file, one `---` document per pose, that `LoadPoseStampedVectorFromYaml` reads back.
 *
 * MoveIt Pro has the vector load but no vector save, so a captured path can now be replayed.
 *
 * Ports:
 *   file_path (input, std::string)                          - absolute path of the YAML file to write.
 *   input     (input, std::vector<geometry_msgs::PoseStamped>) - the path to save.
 */
class SavePoseStampedVectorToYaml final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SavePoseStampedVectorToYaml(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace yaml_behaviors
