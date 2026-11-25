// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "yaml_behaviors/register_yaml_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "yaml_behaviors/append_yaml_list_item.hpp"
#include "yaml_behaviors/write_yaml_value.hpp"

namespace yaml_behaviors
{
void registerYamlBehaviors(BT::BehaviorTreeFactory& factory,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<WriteYamlValue>(factory, "WriteYamlValue", shared_resources);
  moveit_pro::behaviors::registerBehavior<AppendYamlListItem>(factory, "AppendYamlListItem", shared_resources);
}
}  // namespace yaml_behaviors
