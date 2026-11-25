// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "blackboard_behaviors/register_blackboard_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "blackboard_behaviors/get_blackboard_by_key.hpp"
#include "blackboard_behaviors/set_blackboard_by_key.hpp"

namespace blackboard_behaviors
{
void registerBlackboardBehaviors(BT::BehaviorTreeFactory& factory,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<GetBlackboardByKey>(factory, "GetBlackboardByKey", shared_resources);
  moveit_pro::behaviors::registerBehavior<SetBlackboardByKey>(factory, "SetBlackboardByKey", shared_resources);
}
}  // namespace blackboard_behaviors
