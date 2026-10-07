// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "experimental_behaviors/access_interface_value_from_group.hpp"
#include "experimental_behaviors/append_yaml_list_item.hpp"
#include "experimental_behaviors/create_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/create_interface_value.hpp"
#include "experimental_behaviors/get_blackboard_by_key.hpp"
#include "experimental_behaviors/get_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/get_interface_value_from_group.hpp"
#include "experimental_behaviors/get_joint_limits.hpp"
#include "experimental_behaviors/get_pose_stamped_from_topic.hpp"
#include "experimental_behaviors/json_behaviors/create_json.hpp"
#include "experimental_behaviors/json_behaviors/get_json_field.hpp"
#include "experimental_behaviors/json_behaviors/has_json_field.hpp"
#include "experimental_behaviors/json_behaviors/merge_json.hpp"
#include "experimental_behaviors/json_behaviors/receive_json_udp.hpp"
#include "experimental_behaviors/json_behaviors/remove_json_field.hpp"
#include "experimental_behaviors/json_behaviors/send_json_http.hpp"
#include "experimental_behaviors/json_behaviors/send_json_udp.hpp"
#include "experimental_behaviors/json_behaviors/set_json_field.hpp"
#include "experimental_behaviors/publish_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/set_blackboard_by_key.hpp"
#include "experimental_behaviors/trajectory_to_path.hpp"
#include "experimental_behaviors/write_yaml_value.hpp"

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
    moveit_pro::behaviors::registerBehavior<GetJointLimits>(factory, "GetJointLimits", shared_resources);
    moveit_pro::behaviors::registerBehavior<SetBlackboardByKey>(factory, "SetBlackboardByKey", shared_resources);
    moveit_pro::behaviors::registerBehavior<TrajectoryToPath>(factory, "TrajectoryToPath", shared_resources);
    moveit_pro::behaviors::registerBehavior<WriteYamlValue>(factory, "WriteYamlValue", shared_resources);
    moveit_pro::behaviors::registerBehavior<AppendYamlListItem>(factory, "AppendYamlListItem", shared_resources);
    moveit_pro::behaviors::registerBehavior<CreateJson>(factory, "CreateJson", shared_resources);
    moveit_pro::behaviors::registerBehavior<SetJsonField>(factory, "SetJsonField", shared_resources);
    moveit_pro::behaviors::registerBehavior<GetJsonField>(factory, "GetJsonField", shared_resources);
    moveit_pro::behaviors::registerBehavior<RemoveJsonField>(factory, "RemoveJsonField", shared_resources);
    moveit_pro::behaviors::registerBehavior<HasJsonField>(factory, "HasJsonField", shared_resources);
    moveit_pro::behaviors::registerBehavior<MergeJson>(factory, "MergeJson", shared_resources);
    moveit_pro::behaviors::registerBehavior<SendJsonUdp>(factory, "SendJsonUdp", shared_resources);
    moveit_pro::behaviors::registerBehavior<ReceiveJsonUdp>(factory, "ReceiveJsonUdp", shared_resources);
    moveit_pro::behaviors::registerBehavior<SendJsonHttp>(factory, "SendJsonHttp", shared_resources);
  }
};
}  // namespace experimental_behaviors

PLUGINLIB_EXPORT_CLASS(experimental_behaviors::ExperimentalBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
