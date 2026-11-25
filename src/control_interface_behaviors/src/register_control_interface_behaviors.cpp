// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "control_interface_behaviors/register_control_interface_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "control_interface_behaviors/access_interface_value_from_group.hpp"
#include "control_interface_behaviors/create_dynamic_interface_group_values.hpp"
#include "control_interface_behaviors/create_interface_value.hpp"
#include "control_interface_behaviors/get_dynamic_interface_group_values.hpp"
#include "control_interface_behaviors/get_interface_value_from_group.hpp"
#include "control_interface_behaviors/publish_dynamic_interface_group_values.hpp"

namespace control_interface_behaviors
{
void registerControlInterfaceBehaviors(BT::BehaviorTreeFactory& factory,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<CreateDynamicInterfaceGroupValues>(
      factory, "CreateDynamicInterfaceGroupValues", shared_resources);
  moveit_pro::behaviors::registerBehavior<PublishDynamicInterfaceGroupValues>(
      factory, "PublishDynamicInterfaceGroupValues", shared_resources);
  moveit_pro::behaviors::registerBehavior<CreateInterfaceValue>(factory, "CreateInterfaceValue", shared_resources);
  moveit_pro::behaviors::registerBehavior<GetDynamicInterfaceGroupValues>(factory, "GetDynamicInterfaceGroupValues",
                                                                          shared_resources);
  moveit_pro::behaviors::registerBehavior<GetInterfaceValueFromGroup>(factory, "GetInterfaceValueFromGroup",
                                                                      shared_resources);
  moveit_pro::behaviors::registerBehavior<AccessInterfaceValueFromGroup>(factory, "AccessInterfaceValue",
                                                                         shared_resources);
}
}  // namespace control_interface_behaviors
