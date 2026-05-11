// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "experimental_behaviors/access_interface_value_from_group.hpp"
#include "experimental_behaviors/convert_odom_to_pose_stamped.hpp"
#include "experimental_behaviors/convert_pose_stamped_to_odom.hpp"
#include "experimental_behaviors/create_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/create_interface_value.hpp"
#include "experimental_behaviors/get_blackboard_by_key.hpp"
#include "experimental_behaviors/get_bool.hpp"
#include "experimental_behaviors/get_bool_instance.hpp"
#include "experimental_behaviors/get_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/get_empty_instance.hpp"
#include "experimental_behaviors/get_interface_value_from_group.hpp"
#include "experimental_behaviors/get_odom_instance.hpp"
#include "experimental_behaviors/get_odom_latest.hpp"
#include "experimental_behaviors/get_pose_stamped_from_topic.hpp"
#include "experimental_behaviors/pose_to_vectors.hpp"
#include "experimental_behaviors/publish_bool.hpp"
#include "experimental_behaviors/publish_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/set_blackboard_by_key.hpp"
#include "experimental_behaviors/trajectory_to_path.hpp"
#include "experimental_behaviors/vectors_to_pose.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace experimental_behaviors
{
class ExperimentalBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<GetPoseStampedFromTopic>(factory, "GetPoseStampedFromTopic",
                                                                     shared_resources);
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
    moveit_pro::behaviors::registerBehavior<GetBlackboardByKey>(factory, "GetBlackboardByKey", shared_resources);
    moveit_pro::behaviors::registerBehavior<SetBlackboardByKey>(factory, "SetBlackboardByKey", shared_resources);
    moveit_pro::behaviors::registerBehavior<TrajectoryToPath>(factory, "TrajectoryToPath", shared_resources);
    moveit_pro::behaviors::registerBehavior<PoseToVectors>(factory, "PoseToVectors", shared_resources);
    moveit_pro::behaviors::registerBehavior<VectorsToPose>(factory, "VectorsToPose", shared_resources);
    moveit_pro::behaviors::registerBehavior<ConvertOdomToPoseStamped>(factory, "ConvertOdomToPoseStamped",
                                                                       shared_resources);
    moveit_pro::behaviors::registerBehavior<ConvertPoseStampedToOdom>(factory, "ConvertPoseStampedToOdom",
                                                                       shared_resources);
    moveit_pro::behaviors::registerBehavior<GetOdomLatest>(factory, "GetOdomLatest", shared_resources);
    moveit_pro::behaviors::registerBehavior<PublishBool>(factory, "PublishBool", shared_resources);
    moveit_pro::behaviors::registerBehavior<GetBool>(factory, "GetBool", shared_resources);
    moveit_pro::behaviors::registerBehavior<GetBoolInstance>(factory, "GetBoolInstance", shared_resources);
    moveit_pro::behaviors::registerBehavior<GetEmptyInstance>(factory, "GetEmptyInstance", shared_resources);
    moveit_pro::behaviors::registerBehavior<GetOdomInstance>(factory, "GetOdomInstance", shared_resources);
  }
};
}  // namespace experimental_behaviors

PLUGINLIB_EXPORT_CLASS(experimental_behaviors::ExperimentalBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
