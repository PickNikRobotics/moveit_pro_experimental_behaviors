// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "utility_behaviors/register_utility_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "utility_behaviors/select_random_string_from_vector.hpp"
#include "utility_behaviors/stopwatch_check.hpp"
#include "utility_behaviors/stopwatch_log.hpp"

namespace utility_behaviors
{
void registerUtilityBehaviors(BT::BehaviorTreeFactory& factory,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<SelectRandomStringFromVector>(factory, "SelectRandomStringFromVector",
                                                                        shared_resources);
  moveit_pro::behaviors::registerBehavior<StopWatchCheck>(factory, "StopWatchCheck");
  moveit_pro::behaviors::registerBehavior<StopwatchLog>(factory, "StopwatchLog", shared_resources);
}
}  // namespace utility_behaviors
