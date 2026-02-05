#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "experimental_behaviors/access_interface_value_from_group.hpp"
#include "experimental_behaviors/create_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/create_interface_value.hpp"
#include "experimental_behaviors/get_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/get_interface_value_from_group.hpp"
#include "experimental_behaviors/get_pose_stamped_from_topic.hpp"
#include "experimental_behaviors/publish_dynamic_interface_group_values.hpp"

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
  }
};
}  // namespace experimental_behaviors

PLUGINLIB_EXPORT_CLASS(experimental_behaviors::ExperimentalBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
