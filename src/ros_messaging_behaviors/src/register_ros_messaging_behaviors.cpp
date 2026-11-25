// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "ros_messaging_behaviors/register_ros_messaging_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "ros_messaging_behaviors/get_pose_stamped_from_topic.hpp"

namespace ros_messaging_behaviors
{
void registerRosMessagingBehaviors(BT::BehaviorTreeFactory& factory,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<GetPoseStampedFromTopic>(factory, "GetPoseStampedFromTopic", shared_resources);
}
}  // namespace ros_messaging_behaviors
