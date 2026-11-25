// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "yaml_behaviors/register_yaml_behaviors.hpp"

#include <moveit_pro_behavior_interface/save_to_yaml.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_common/utils/yaml_parsing_tools.hpp>
#include <nav_msgs/msg/path.hpp>

#include "yaml_behaviors/append_yaml_list_item.hpp"
#include "yaml_behaviors/read_yaml_list.hpp"
#include "yaml_behaviors/read_yaml_value.hpp"
#include "yaml_behaviors/save_point_stamped_vector_to_yaml.hpp"
#include "yaml_behaviors/save_pose_stamped_vector_to_yaml.hpp"
#include "yaml_behaviors/write_yaml_value.hpp"

// YAML conversion for nav_msgs/Path, which SaveToYaml<nav_msgs::msg::Path> (SavePathToYaml) needs.
ROS_MESSAGE_YAML_PARSER(nav_msgs, Path)

namespace yaml_behaviors
{
void registerYamlBehaviors(BT::BehaviorTreeFactory& factory,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<WriteYamlValue>(factory, "WriteYamlValue", shared_resources);
  moveit_pro::behaviors::registerBehavior<AppendYamlListItem>(factory, "AppendYamlListItem", shared_resources);
  moveit_pro::behaviors::registerBehavior<ReadYamlValue>(factory, "ReadYamlValue", shared_resources);
  moveit_pro::behaviors::registerBehavior<ReadYamlList>(factory, "ReadYamlList", shared_resources);
  moveit_pro::behaviors::registerBehavior<moveit_pro::behaviors::SaveToYaml<nav_msgs::msg::Path>>(
      factory, "SavePathToYaml", shared_resources);
  moveit_pro::behaviors::registerBehavior<SavePointStampedVectorToYaml>(factory, "SavePointStampedVectorToYaml",
                                                                        shared_resources);
  moveit_pro::behaviors::registerBehavior<SavePoseStampedVectorToYaml>(factory, "SavePoseStampedVectorToYaml",
                                                                       shared_resources);
}
}  // namespace yaml_behaviors
