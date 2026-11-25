// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "tf_behaviors/register_tf_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "tf_behaviors/monitor_transform_z_below_threshold.hpp"
#include "tf_behaviors/publish_static_transform.hpp"

namespace tf_behaviors
{
void registerTfBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<MonitorTransformZBelowThreshold>(factory, "MonitorTransformZBelowThreshold",
                                                                           shared_resources);
  moveit_pro::behaviors::registerBehavior<PublishStaticTransform>(factory, "PublishStaticTransform", shared_resources);
}
}  // namespace tf_behaviors
