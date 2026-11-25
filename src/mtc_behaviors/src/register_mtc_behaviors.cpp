// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "mtc_behaviors/register_mtc_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "mtc_behaviors/get_mtc_solution_cost.hpp"
#include "mtc_behaviors/set_collisions_between_groups.hpp"

namespace mtc_behaviors
{
void registerMtcBehaviors(BT::BehaviorTreeFactory& factory,
                          const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<GetMtcSolutionCost>(factory, "GetMtcSolutionCost", shared_resources);
  moveit_pro::behaviors::registerBehavior<SetCollisionsBetweenGroups>(factory, "SetCollisionsBetweenGroups",
                                                                      shared_resources);
}
}  // namespace mtc_behaviors
