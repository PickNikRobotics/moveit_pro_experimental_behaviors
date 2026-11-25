// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>

namespace tf_behaviors
{
/**
 * @brief Registers every Behavior in the tf_behaviors folder with @p factory.
 * @param factory The factory to register the Behaviors with.
 * @param shared_resources The Behavior context handed to each Behavior.
 */
void registerTfBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);
}  // namespace tf_behaviors
