// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "hardware_behaviors/register_hardware_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "hardware_behaviors/set_hardware_component_state.hpp"

namespace hardware_behaviors
{
void registerHardwareBehaviors(BT::BehaviorTreeFactory& factory,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<SetHardwareComponentState>(factory, "SetHardwareComponentState",
                                                                     shared_resources);
}
}  // namespace hardware_behaviors
