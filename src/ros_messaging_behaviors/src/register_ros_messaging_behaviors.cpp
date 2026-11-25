// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "ros_messaging_behaviors/register_ros_messaging_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "ros_messaging_behaviors/get_pose_stamped_from_topic.hpp"
#include "ros_messaging_behaviors/get_string_from_topic.hpp"
#include "ros_messaging_behaviors/maintain_topic_subscription.hpp"
#include "ros_messaging_behaviors/publish_pose_stamped.hpp"
#include "ros_messaging_behaviors/publish_twist_stamped.hpp"

namespace ros_messaging_behaviors
{
void registerRosMessagingBehaviors(BT::BehaviorTreeFactory& factory,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<GetPoseStampedFromTopic>(factory, "GetPoseStampedFromTopic", shared_resources);
  moveit_pro::behaviors::registerBehavior<GetStringFromTopic>(factory, "GetStringFromTopic", shared_resources);
  moveit_pro::behaviors::registerBehavior<MaintainTopicSubscription>(factory, "MaintainTopicSubscription",
                                                                     shared_resources);
  moveit_pro::behaviors::registerBehavior<PublishPoseStamped>(factory, "PublishPoseStamped", shared_resources);
  moveit_pro::behaviors::registerBehavior<PublishTwistStamped>(factory, "PublishTwistStamped", shared_resources);
}
}  // namespace ros_messaging_behaviors
