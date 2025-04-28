#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include "experimental_behaviors/get_pose_stamped_from_topic.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace experimental_behaviors
{
class ExperimentalBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<GetPoseStampedFromTopic>(factory, "GetPoseStampedFromTopic",
                                                                        shared_resources);
  }
};
}  // namespace experimental_behaviors

PLUGINLIB_EXPORT_CLASS(experimental_behaviors::ExperimentalBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
