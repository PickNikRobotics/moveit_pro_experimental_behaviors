// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "mobile_base_behaviors/register_mobile_base_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "mobile_base_behaviors/teleoperate_base.hpp"

namespace mobile_base_behaviors
{
void registerMobileBaseBehaviors(BT::BehaviorTreeFactory& factory,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<TeleoperateBase>(factory, "TeleoperateBase", shared_resources);
}
}  // namespace mobile_base_behaviors
